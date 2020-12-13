//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_

#include <stdint.h>
#include <stdio.h>
#define DEFAULT_RING_BUFFER_SIZE 128



template <typename T>
class RingBuffer {
 public:
  RingBuffer();
  explicit RingBuffer(int size);
  ~RingBuffer();

  bool isFull();
  bool isEmpty();

  bool insert(const T& element);
  T& remove();
  T& peek();


 private:
  int32_t size;
  int32_t head;
  int32_t tail;
  T * data;

  void init(int size);
  void deinit();
};



template<typename T>
RingBuffer<T>::RingBuffer() {
  init(DEFAULT_RING_BUFFER_SIZE);
}

template<typename T>
RingBuffer<T>::RingBuffer(int size) {
  init(size);
}

template<typename T>
void RingBuffer<T>::init(int size) {
  this->size = size;
  this->head = 0;
  this->tail = 0;
  this->data = new T[size];
}
template<typename T>
void RingBuffer<T>::deinit() {
  this->size = 0;
  this->head = 0;
  this->tail = 0;
  
  if (data != NULL)
    delete data;
  data = NULL;
}
template<typename T>
RingBuffer<T>::~RingBuffer() {
  deinit();
}
template<typename T>
bool RingBuffer<T>::isFull() {
  return (tail + 1 == head) || ((tail + 1) == size && head == 0);
}
template<typename T>
bool RingBuffer<T>::isEmpty() {
  return head == tail;
}
template<typename T>
bool RingBuffer<T>::insert(const T& element) {
  if (isFull()) {
    return false;
  }
  data[tail] = element;
  tail = (tail + 1 == size) ? 0 : tail + 1;
  return true;
}
template<typename T>
T& RingBuffer<T>::remove() {
  if (isEmpty()) {
    // safe guard?
    // return nullptr;
  }
  T& ret = data[head];
  head = (head + 1 == size) ? 0 : head + 1;

  return ret;
}

template<typename T>
T& RingBuffer<T>::peek() {
  if (isEmpty()) {
    // return nullptr;
  }
  return data[head];
}


#endif //MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
