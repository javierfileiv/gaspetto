#include "MessageQueue.h"
#include <stdint.h>

MessageQueue::MessageQueue()
    : head_(0), tail_(0), count_(0), buttonPool_(), printPool_(), nrfTxPool_(),
      nrfRxPool_() {}

template <typename T, typename... Args>
T *MessageQueue::allocate(Args... args) {
  if constexpr (std::is_same_v<T, MessageButton>) {
    MessageButton *msg = buttonPool_.allocate();
    if (msg != nullptr) {
      new (msg) MessageButton(args...);
    }
    return msg;
  } else if constexpr (std::is_same_v<T, MessagePrint>) {
    MessagePrint *msg = printPool_.allocate();
    if (msg != nullptr) {
      new (msg) MessagePrint(args...);
    }
    return msg;
  } else if constexpr (std::is_same_v<T, MessageNrfTx>) {
    MessageNrfTx *msg = nrfTxPool_.allocate();
    if (msg != nullptr) {
      new (msg) MessageNrfTx(args...);
    }
    return msg;
  } else if constexpr (std::is_same_v<T, MessageNrfRx>) {
    MessageNrfRx *msg = nrfRxPool_.allocate();
    if (msg != nullptr) {
      new (msg) MessageNrfRx(args...);
    }
    return msg;
  }
  return nullptr; // Tipo no reconocido
}

template MessageButton *MessageQueue::allocate<MessageButton, uint8_t>(uint8_t);
template MessagePrint *
MessageQueue::allocate<MessagePrint, const char *>(const char *);
template MessageNrfTx *
MessageQueue::allocate<MessageNrfTx, const char *>(const char *);
template MessageNrfRx *
MessageQueue::allocate<MessageNrfRx, const char *>(const char *);
template MessagePrintTimed *
MessageQueue::allocate<MessagePrintTimed, const char *,  unsigned long>(const char *, unsigned long);

bool MessageQueue::enqueue(Message *message) {
  if (!isFull()) {
    buffer_[tail_] = message;
    tail_ = (tail_ + 1) % QUEUE_CAPACITY;
    count_++;
    return true;
  }
  return false;
}

void MessageQueue::release(Message *message) {
  switch (message->getType()) {
  case MessageType::BUTTON:
    if (MessageButton *btnMsg = static_cast<MessageButton *>(message)) {
      buttonPool_.release(btnMsg);
    }
    break;
  case MessageType::PRINT:
    if (MessagePrint *printMsg = static_cast<MessagePrint *>(message)) {
      printPool_.release(printMsg);
    }
    break;
    case MessageType::NRF_TX:
    if (MessageNrfTx *nrfTxMsg = static_cast<MessageNrfTx *>(message)) {
      nrfTxPool_.release(nrfTxMsg);
    }
    break;
    case MessageType::NRF_RX:
    if (MessageNrfRx *nrfRxMsg = static_cast<MessageNrfRx *>(message)) {
      nrfRxPool_.release(nrfRxMsg);
    }
    break;
    default:
    break;
  }
}

bool MessageQueue::isEmpty() const { return count_ == 0; }

bool MessageQueue::isFull() const { return count_ == QUEUE_CAPACITY; }

Message *MessageQueue::peek(int index) const {
  if (index >= 0 && index < count_) {
    return buffer_[(head_ + index) % QUEUE_CAPACITY];
  }
  return nullptr;
}

bool MessageQueue::dequeue(Message *messageToRemove) {
  for (int i = 0; i < count_; ++i) {
    if (buffer_[(head_ + i) % QUEUE_CAPACITY] == messageToRemove) {
      // Shift elementos para eliminar
      int indexToRemove = (head_ + i) % QUEUE_CAPACITY;
      if (indexToRemove >= head_) {
        for (int j = indexToRemove; j < tail_; ++j) {
          buffer_[j] = buffer_[j + 1];
        }
        tail_ = (tail_ == 0) ? QUEUE_CAPACITY - 1 : tail_ - 1;
      } else {
        for (int j = indexToRemove; j > head_; --j) {
          buffer_[j] = buffer_[j - 1];
        }
        head_ = (head_ + 1) % QUEUE_CAPACITY;
      }
      count_--;
      return true;
    }
  }
  return false;
}

Message *MessageQueue::dequeue() {
  if (!isEmpty()) {
    Message *message = buffer_[head_];
    head_ = (head_ + 1) % QUEUE_CAPACITY;
    count_--;
    return message;
  }
  return nullptr;
}
