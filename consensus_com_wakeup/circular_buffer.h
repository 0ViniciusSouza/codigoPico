#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

template <class T, int buffer_size = 16>

class circular_buffer {
    T buffer[buffer_size]; //storage
    int head; //index of next element to write
    int tail; //index of next element to read
    int count; //counter of valid elements in the buffer
  public:
    circular_buffer() : head {0}, tail{0}, count{0} {};
    ~circular_buffer() {};
    int capacity() { return buffer_size; }
    int size() { return count; }
    bool is_empty() { return count == 0; }
    bool is_full() { return count == buffer_size; }
    
    T peak() { return buffer[tail]; } // inspect but keep
    //value invalid if buffer empty
    
    void put(T elem) {//add an element to the buffer
      if(!is_full()) {
        buffer[head++] = elem;
        if(head == buffer_size) head = 0;
        count++;
      } //does nothing if buffer full
    }
    
    T take() {//remove an element from the buffer
      T tmp_tail = buffer[tail]; //invalid if buffer empty
      if(!is_empty()){
        count--; tail++;
        if(tail == buffer_size) tail = 0;
      }
      return tmp_tail;
    }
};

#endif //CIRCULAR_BUFFER_H
