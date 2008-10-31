#ifndef __MEMORY_H__
#define __MEMORY_H__

#include <cstring>

// THIS IS NOT A REAL HEAP, this just allocates the next spot, then we should free EVERYTHING else this won't work.

#define HEAP_CNT 512 // how many things we can have in the heap

void initMEM2();
void * mallocMEM2(size_t);
void freeMEM2(void*);

typedef struct {
	unsigned int * ptr;
	size_t size;
} MEM2Heap;

extern MEM2Heap MEM2HeapArray[HEAP_CNT];

#endif
