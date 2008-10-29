#include "memory.h"

unsigned int MEM2StorPtr = 0x91000000; // we can probably nip this closer to x900000000 as we work
MEM2Heap MEM2HeapArray[HEAP_CNT];

void initMEM2()
{
	memset(MEM2HeapArray,0,sizeof(MEM2Heap)*HEAP_CNT);
}

void * mallocMEM2(size_t size)
{
	int i = 0;
	for(; i < HEAP_CNT; i++){
		if ( MEM2HeapArray[i].ptr == 0 ){
			// check how much size is in this heap
			if ( i > 0 ){			
				MEM2HeapArray[i].ptr = (unsigned int*)(MEM2HeapArray[i-1].ptr + MEM2HeapArray[i-1].size);			
				MEM2HeapArray[i].size = size;
			}
			else{
				MEM2HeapArray[i].ptr = (unsigned int*)MEM2StorPtr;
				MEM2HeapArray[i].size = size;
			}
			break;
		}
	}

	if ( i == HEAP_CNT ) return 0;
	else return MEM2HeapArray[i].ptr;
}

void freeMEM2(void * ptr)
{
	for(int i = 0; i < HEAP_CNT; i++){
		if ( MEM2HeapArray[i].ptr == ptr){
			MEM2HeapArray[i].ptr = 0;
			MEM2HeapArray[i].size = 0;		
		}	
	}
}

