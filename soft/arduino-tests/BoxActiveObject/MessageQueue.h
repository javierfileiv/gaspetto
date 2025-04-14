#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include "Message.h"
#include "MessageButton.h"
#include "MessagePrint.h"
#include "MessageNrfTx.h"
#include "MessageNrfRx.h"
#include "MessagePool.h"
#include "MessagePrintTimed.h"

class MessageQueue {
public:
  static const int QUEUE_CAPACITY = 10;
  static const int BUTTON_POOL_SIZE = 5;
  static const int PRINT_POOL_SIZE = 5;
  static const int TIMED_PRINT_POOL_SIZE = 5;
  static const int NRF_TX_POOL_SIZE = 3;
  static const int NRF_RX_POOL_SIZE = 3;

  MessageQueue();
  template <typename T, typename... Args>
  T* allocate(Args... args);
  bool enqueue(Message* message);
  Message* dequeue();
  bool dequeue(Message* messageToRemove);
  Message* peek(int index) const;
  int getCount() const { return count_; }
  void release(Message* message);
  bool isEmpty() const;
  bool isFull() const;
private:
  Message* buffer_[QUEUE_CAPACITY];
  int head_;
  int tail_;
  int count_;
  ObjectPool<MessageButton, BUTTON_POOL_SIZE> buttonPool_;
  ObjectPool<MessagePrint, PRINT_POOL_SIZE> printPool_;
  ObjectPool<MessageNrfTx, NRF_TX_POOL_SIZE> nrfTxPool_;
  ObjectPool<MessageNrfRx, NRF_RX_POOL_SIZE> nrfRxPool_;
  ObjectPool<MessagePrintTimed, TIMED_PRINT_POOL_SIZE> timedPrintPool_;
};

#endif