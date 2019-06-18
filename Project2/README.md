## Project2

### Design of cache organization

![cache_architecture](./cache_architecture.png)

* Four-way set-associative cache with 16 cache sets in total
* Unified cache used for both instruction and memory access
* 16 bytes in each cache line with data

#### cache_line

```c
typedef struct cache_line {
    unsigned int data[4];     
    unsigned int tag:27;      
    unsigned int dirty:1;   
    unsigned int valid:1;    
    unsigned int ref_count:19;     
    struct cache_line *next;    
} cache_line_t;
```

* data: data in cache line with total size of 4 bytes 
* tag: marked for addressing
* dirty:  whether the cache_line has been written
* valid:  whether the cache_line has been in use
* ref_count: reference count 
* next: next cache_line in the same cache set

#### cache_set

```c
typedef struct cache_set {
    cache_line_t *head;        
    cache_line_t *tail;        
    int n;                     
} cache_set_t;
```

* Queue struct is used for each cache_set for implementation of FIFO

#### cache 

```c
typedef struct cache {
    cache_set_t sets[16];      
    unsigned int enable;       
    unsigned int access;    
    unsigned int hit;          
    unsigned int miss;     
    unsigned int replace;      
    unsigned int wb;            
} cache_t;
```

   #### FIFO Replacement Implementation

* using a queue to implement FIFO algorithm

```c
void cache_set_enqueue(cache_line_t* cache_line) {
	// enqueue cache_line to the cache_set.
}
void cache_set_dequeue(cache_line_t* cache_line) {
  // dequeue cache_line(the oldest) from the cache_set.
}
```

#### cache access

- cache access will cause hit or miss, and miss means you need to create a cache line corresponding to the miss data and add it to the cache. which may cause write back.

#### write back and flush

* when you add a cache line to a full cache set, or when you need to flush your cache,  the replaced cache line with true dirty bit will be write back to memory.

#### Read and Write

* read and write are two specific access operations in our project.

```c
int cache_read(md_addr_t addr, word_t *word) {
  // access addr and move it into cache if not
  // read data at addr from cache and store it into word
  // return cycles
}
int cache_write(md_addr_t addr, word_t *word) {
  // access addr and move it into cache if not
  // write data stored in word at addr in the cache, and make dirty bit to true  
  // return cycles
}
```

### Statistics

[statistic_result.txt](./statistic_result.txt)

```
Clock Cycles: 14120472
Memory Accesses: 11068194
Memory Hits: 10729052
Memory Misses: 339142
Line Replacements: 339078
Line Write-backs: 20214
```

