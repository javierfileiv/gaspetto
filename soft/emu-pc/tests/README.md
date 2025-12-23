# Tests - Unit Testing Suite

Comprehensive unit test suite using GoogleTest and GoogleMock for testing embedded firmware without hardware dependencies.

## Overview

This test suite enables:
- **Hardware-independent testing**: Mock all peripherals and sensors
- **Fast feedback**: Tests run on PC in milliseconds
- **Behavior verification**: Validate logic without hardware
- **Regression prevention**: Automated test execution
- **TDD support**: Write tests before implementation

## Structure

```
tests/
├── CMakeLists.txt                    # Test build configuration
├── main.cpp                          # GoogleTest main entry point
├── fixture.h / fixture.cpp           # Common test fixture
├── gaspetto_car/
│   ├── test_system.cpp               # System integration tests
│   ├── test_radio_controller.cpp     # Radio tests
│   └── test_movement_controller.cpp  # Movement tests
└── mocks/
    ├── mock_Arduino.h / .cpp         # Arduino function mocks
    ├── mock_MotorControl.h / .cpp    # Motor control mock
    ├── mock_IMUOrientation.h / .cpp  # IMU sensor mock
    └── mock_RF24.h / .cpp            # RF24 radio mock
```

## Testing Framework

### GoogleTest
- Industry-standard C++ testing framework
- Rich assertion library
- Test fixtures and parameterized tests
- Test discovery and execution

### GoogleMock
- C++ mocking framework
- Expectation setting and verification
- Strict/nice/naggy mock modes
- Sequence and cardinality checking

## Test Fixture

The `Fixture` class provides a complete test environment:

```cpp
class Fixture : public ::testing::Test {
public:
    Fixture()
        : radioController(mockRF24, &eventQueue, ...)
        , mockMotorControl()
        , mockIMU()
        , carMovementController(mockMotorControl, mockIMU)
        , eventQueue()
        , ctx{ ... }
        , car(ctx)
    {
        // Initialize mocks with default behavior
    }

    void SetUp() override {
        // Called before each test
        expect_car_init();
        car.init(StateId::IDLE);
    }

protected:
    // All dependencies available to tests
    GaspettoCar car;
    MockMotorControl mockMotorControl;
    MockIMUOrientation mockIMU;
    MockRF24 mockRF24;
    EventQueue eventQueue;
    // ...
};
```

## Mock Objects

### MockMotorControl

Mocks the motor driver interface:

```cpp
class MockMotorControl : public MotorControlInterface {
public:
    MOCK_METHOD(void, _init, (uint32_t pwm_freq));
    MOCK_METHOD(void, _setMotorLeft, (bool forward, uint8_t speed_percent));
    MOCK_METHOD(void, _setMotorRight, (bool forward, uint8_t speed_percent));
    MOCK_METHOD(void, _stopLeft, ());
    MOCK_METHOD(void, _stopRight, ());
};
```

### MockIMUOrientation

Mocks the IMU sensor interface:

```cpp
class MockIMUOrientation : public IMUOrientationInterface {
public:
    MOCK_METHOD(bool, _begin, (uint8_t addr, TwoWire* wire));
    MOCK_METHOD(void, _update, ());
    MOCK_METHOD(float, _getYaw, ());
    MOCK_METHOD(bool, _isReady, ());
};
```

### MockRF24

Mocks the RF24 radio transceiver:

```cpp
class MockRF24 : public RF24 {
public:
    MOCK_METHOD(bool, _begin, ());
    MOCK_METHOD(bool, _write, (const void* buf, uint8_t len));
    MOCK_METHOD(bool, _available, ());
    MOCK_METHOD(void, _read, (void* buf, uint8_t len));
    MOCK_METHOD(void, _startListening, ());
    MOCK_METHOD(void, _stopListening, ());
    // ...
};
```

## Writing Tests

### Basic Test Structure

```cpp
TEST_F(Fixture, TestName) {
    // Arrange - Set up expectations
    EXPECT_CALL(mockMotorControl, _setMotorLeft(true, 80));
    EXPECT_CALL(mockMotorControl, _setMotorRight(true, 80));

    // Act - Perform action
    Event cmd{EventId::ACTION, CommandId::MOTOR_FORWARD, 80};
    eventQueue.sendEvent(&cmd);
    car.processEvents();

    // Assert - Verify results
    ASSERT_EQ(car.getCurrentState()->getId(), StateId::PROCESSING);
}
```

### Expectation Setting

#### Exact Match
```cpp
EXPECT_CALL(mockMotor, _setMotorLeft(true, 80));
```

#### Any Arguments
```cpp
using testing::_;
EXPECT_CALL(mockMotor, _setMotorLeft(_, _));
```

#### Return Values
```cpp
EXPECT_CALL(mockIMU, _begin(_, _))
    .WillOnce(Return(true));
```

#### Call Count
```cpp
EXPECT_CALL(mockMotor, _stopLeft())
    .Times(1);  // Exactly once

EXPECT_CALL(mockMotor, _setMotorLeft(_, _))
    .Times(AtLeast(1));  // One or more times
```

#### Sequence Checking
```cpp
testing::InSequence seq;  // Enforce order

EXPECT_CALL(mockMotor, _setMotorLeft(true, 80));
EXPECT_CALL(mockMotor, _setMotorRight(true, 80));
// Must be called in this order
```

## Test Categories

### State Machine Tests

Verify state transitions and event handling:

```cpp
TEST_F(Fixture, IdleToProcessingTransition) {
    ASSERT_EQ(car.getCurrentState()->getId(), StateId::IDLE);

    Event evt{EventId::ACTION, CommandId::MOTOR_FORWARD};
    expect_move_forward(80, 80);

    RxRadioEvent(evt);

    ASSERT_EQ(car.getCurrentState()->getId(), StateId::PROCESSING);
}
```

### Radio Communication Tests

Verify command reception and transmission:

```cpp
TEST_F(Fixture, ReceivesRadioCommand) {
    Event cmd{EventId::ACTION, CommandId::MOTOR_STOP};

    EXPECT_CALL(mockRF24, _available())
        .WillOnce(Return(true));
    EXPECT_CALL(mockRF24, _read(_, sizeof(Event)))
        .WillOnce(SetArgReferee<0>(cmd));

    radioController.processRadioEvent();

    Event* received = eventQueue.getEvent();
    ASSERT_NE(received, nullptr);
    ASSERT_EQ(received->commandId, CommandId::MOTOR_STOP);
}
```

### Movement Controller Tests

Verify motor control logic:

```cpp
TEST_F(Fixture, ForwardMovement) {
    EXPECT_CALL(mockMotorControl, _setMotorLeft(true, 75));
    EXPECT_CALL(mockMotorControl, _setMotorRight(true, 75));

    movementController.setMotorSpeeds(true, 75, true, 75);
}

TEST_F(Fixture, StopMovement) {
    EXPECT_CALL(mockMotorControl, _stopLeft());
    EXPECT_CALL(mockMotorControl, _stopRight());

    movementController.stop();
}
```

### PID Control Tests

Verify PID behavior with IMU feedback:

```cpp
TEST_F(Fixture, StraightDrivingPID) {
    // Set up IMU to return stable heading
    EXPECT_CALL(mockIMU, _getYaw())
        .WillRepeatedly(Return(180.0f));

    movementController.startStraightDriving(true, 70, 180.0f);

    // Update should maintain speed without correction
    movementController.updateMovement();

    // Verify motors maintain equal speed
}
```

## Test Utilities

### Fixture Helper Methods

```cpp
// Expect complete car initialization
void expect_car_init();

// Expect motor initialization
void expect_motor_control_init();

// Expect movement commands
void expect_move_forward(uint32_t leftSpeed, uint32_t rightSpeed);
void expect_move_backward(uint32_t leftSpeed, uint32_t rightSpeed);
void expect_turn_left(uint32_t leftSpeed, uint32_t rightSpeed);
void expect_turn_right(uint32_t leftSpeed, uint32_t rightSpeed);

// Expect stop commands
void expect_stop_motor_left();
void expect_stop_motor_right();
void expect_both_motors_stop();

// Radio event injection
void RxRadioEvent(Event evt);
void expect_transmit_event(Event evt);
```

### Using Helper Methods

```cpp
TEST_F(Fixture, ComplexSequence) {
    expect_move_forward(80, 80);
    Event cmd1{EventId::ACTION, CommandId::MOTOR_FORWARD, 80};
    RxRadioEvent(cmd1);

    expect_turn_left(50, 80);
    Event cmd2{EventId::ACTION, CommandId::MOTOR_LEFT, 65};
    RxRadioEvent(cmd2);

    expect_both_motors_stop();
    Event cmd3{EventId::ACTION, CommandId::MOTOR_STOP};
    RxRadioEvent(cmd3);
}
```

## Running Tests

### Build and Run All Tests

```bash
cd build-tests
cmake -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release ..
make utest
./tests/utest
```

### Using Build Script

```bash
../../tools/build-gaspetto-utest
```

### Run Specific Tests

```bash
# Run only movement controller tests
./tests/utest --gtest_filter="*MovementController*"

# Run only radio tests
./tests/utest --gtest_filter="*Radio*"

# Exclude specific tests
./tests/utest --gtest_filter=-"*Slow*"
```

### With Color Output

```bash
./tests/utest --gtest_color=yes
```

### Verbose Output

```bash
# Show all GMock warnings
GMOCK_VERBOSE=info ./tests/utest
```

## Test Output

### Successful Test Run

```
[==========] Running 15 tests from 4 test suites.
[----------] 5 tests from GaspettoCarTest
[ RUN      ] GaspettoCarTest.InitializesCorrectly
[       OK ] GaspettoCarTest.InitializesCorrectly (0 ms)
[ RUN      ] GaspettoCarTest.HandlesForwardCommand
[       OK ] GaspettoCarTest.HandlesForwardCommand (1 ms)
...
[==========] 15 tests from 4 test suites ran. (42 ms total)
[  PASSED  ] 15 tests.
```

### Failed Test

```
[ RUN      ] GaspettoCarTest.HandlesInvalidCommand
/path/to/test.cpp:123: Failure
Expected: mockMotor._stopLeft() to be called
  Actual: never called
[  FAILED  ] GaspettoCarTest.HandlesInvalidCommand (2 ms)
```

## Debugging Tests

### GDB

```bash
gdb --args ./tests/utest --gtest_filter="*FailingTest*"
(gdb) break test_system.cpp:123
(gdb) run
```

### Print Debugging

```cpp
TEST_F(Fixture, DebugTest) {
    std::cout << "Current state: " << car.getCurrentState()->getId() << std::endl;

    Event* evt = eventQueue.getEvent();
    if (evt) {
        std::cout << "Event: " << evt->eventId << ", " << evt->commandId << std::endl;
    }
}
```

## Mock Behavior Types

### StrictMock
Fails on any unexpected call:
```cpp
testing::StrictMock<MockMotorControl> strictMock;
// Must set expectations for ALL calls
```

### NiceMock
Ignores unexpected calls:
```cpp
testing::NiceMock<MockMotorControl> niceMock;
// Only verify calls you care about
```

### Regular Mock
Warns on unexpected calls:
```cpp
MockMotorControl regularMock;  // Default behavior
```

## Best Practices

### 1. Test One Thing
```cpp
// Good - Tests one behavior
TEST_F(Fixture, ForwardCommandSetsMotors) {
    expect_move_forward(80, 80);
    // ...
}

// Bad - Tests multiple behaviors
TEST_F(Fixture, AllMovementCommands) {
    // Tests forward, backward, turn, stop...
}
```

### 2. Use Descriptive Names
```cpp
// Good
TEST_F(Fixture, TransitionsToProcessingStateOnActionEvent)

// Bad
TEST_F(Fixture, Test1)
```

### 3. Arrange-Act-Assert
```cpp
TEST_F(Fixture, Example) {
    // Arrange - Set up test conditions
    EXPECT_CALL(mock, method());

    // Act - Perform the action
    systemUnderTest.doSomething();

    // Assert - Verify results
    ASSERT_EQ(result, expected);
}
```

### 4. Independent Tests
```cpp
// Each test should be independent
// Don't rely on execution order
// Reset state in SetUp/TearDown
```

### 5. Test Behaviors, Not Implementation
```cpp
// Good - Tests observable behavior
TEST_F(Fixture, StopCommandStopsMotors) {
    expect_both_motors_stop();
    // ...
}

// Bad - Tests internal implementation details
TEST_F(Fixture, StopCommandSetsInternalFlag) {
    // ...
    ASSERT_TRUE(controller.internalStopFlag);
}
```

## Coverage

### Generate Coverage Report

```bash
cmake -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_CXX_FLAGS="--coverage" ..
make utest
./tests/utest
lcov --capture --directory . --output-file coverage.info
genhtml coverage.info --output-directory coverage_report
```

### View Coverage

```bash
firefox coverage_report/index.html
```

## Continuous Integration

Example CI configuration:

```yaml
# .gitlab-ci.yml
test:
  stage: test
  script:
    - mkdir build && cd build
    - cmake -DBUILD_TESTS=ON ..
    - make utest
    - ./tests/utest --gtest_output=xml:test_results.xml
  artifacts:
    reports:
      junit: build/test_results.xml
```

## Troubleshooting

### Linker Errors
- Ensure all source files are in CMakeLists.txt
- Check for multiple definitions
- Verify mock implementations

### Unexpected Mock Calls
- Use `EXPECT_CALL` not `ON_CALL` for verification
- Check call order with `InSequence`
- Use `Times(0)` to forbid calls

### Tests Hang
- Check for infinite loops in mocks
- Verify event queue isn't stuck
- Add timeout to test fixture

## References

- [GoogleTest Primer](https://google.github.io/googletest/primer.html)
- [GoogleMock for Dummies](https://google.github.io/googletest/gmock_for_dummies.html)
- [GoogleMock Cheat Sheet](https://google.github.io/googletest/gmock_cheat_sheet.html)
