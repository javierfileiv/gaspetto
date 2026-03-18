#include "mock_MovementController.h"

MockMovementController::MockMovementController()
        : MockBase<MockMovementController>()
{
    set_instance(this);
}

MockMovementController::~MockMovementController()
{
    clear_instance(this);
}
