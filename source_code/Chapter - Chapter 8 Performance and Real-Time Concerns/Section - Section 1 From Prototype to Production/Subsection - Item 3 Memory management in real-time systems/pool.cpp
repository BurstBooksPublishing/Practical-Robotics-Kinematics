#include <atomic>
#include <cstddef>
#include <new>
#include <pthread.h>

template<typename T, std::size_t N>
class FixedPool {
public:
  FixedPool() {
    // build free list in-place (no heap calls)
    for(std::size_t i=0;i<N;++i) nodes_[i].next = &nodes_[i+1];
    nodes_[N-1].next = nullptr;
    head_.store(&nodes_[0], std::memory_order_relaxed);
  }

  // RT-safe allocate: pop free-list
  T* allocate() {
    Node* old = head_.load(std::memory_order_relaxed);
    while(old) {
      Node* next = old->next;
      if(head_.compare_exchange_weak(old,next,std::memory_order_acq_rel))
        return reinterpret_cast<T*>(&old->storage);
    }
    return nullptr; // pool exhausted; handle in control initialization
  }

  // RT-safe deallocate: push free-list
  void deallocate(T* obj) {
    Node* node = reinterpret_cast<Node*>(obj);
    Node* old = head_.load(std::memory_order_relaxed);
    do {
      node->next = old;
    } while(!head_.compare_exchange_weak(old,node,std::memory_order_release));
  }

private:
  union Node {
    alignas(T) char storage[sizeof(T)];
    Node* next;
  };
  Node nodes_[N];
  std::atomic<Node*> head_;
};