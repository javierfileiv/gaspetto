#ifndef MESSAGE_POOL_H
#define MESSAGE_POOL_H

template <typename T, int SIZE>
class ObjectPool {
public:
  ObjectPool();
  T* allocate();
  void release(T* obj);
private:
  alignas(T) unsigned char pool_buffer_[sizeof(T) * SIZE]; // Buffer estático alineado
  T* pool_[SIZE];
  bool inUse_[SIZE];
  int freeIndex_ = 0;
};

template <typename T, int SIZE>
ObjectPool<T, SIZE>::ObjectPool() {
  for (int i = 0; i < SIZE; ++i) {
    pool_[i] = reinterpret_cast<T*>(&pool_buffer_[i * sizeof(T)]);
    inUse_[i] = false;
  }
}

template <typename T, int SIZE>
T* ObjectPool<T, SIZE>::allocate() {
  for (int i = 0; i < SIZE; ++i) {
    if (!inUse_[i]) {
      inUse_[i] = true;
      return pool_[i];
    }
  }
  return nullptr; // Pool is full
}

template <typename T, int SIZE>
void ObjectPool<T, SIZE>::release(T* obj) {
  for (int i = 0; i < SIZE; ++i) {
    if (pool_[i] == obj && inUse_[i]) {
      inUse_[i] = false;
      return;
    }
  }
}

#endif