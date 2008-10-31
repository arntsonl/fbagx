#include "memory.h"

unsigned int MEM2StorPtr = 0x90000020; // Bushing let me know the first 0x20 bytes are used for the audio in MEM2
// so thats actually where the TP hack loader resides. So as long as we give the 0x20 a break, we can use the rest!


// This is not staying constant
MEM2Heap MEM2HeapArray[HEAP_CNT];

void initMEM2()
{
	memset(MEM2HeapArray,0,sizeof(MEM2Heap)*HEAP_CNT);
}

void * mallocMEM2(size_t size)
{
	unsigned int * rtn = (unsigned int*)MEM2StorPtr;
	MEM2StorPtr = MEM2StorPtr + size;
	if ( MEM2StorPtr > 0x93FFFFFF ) {
		// Whoops, we overflowed, give us back the old pointer and a null pointer
		MEM2StorPtr = (unsigned int)rtn;
		return 0;	
	}
	return rtn;
	/*
	int i = 0;
	for(; i < HEAP_CNT; i++){
		if ( MEM2HeapArray[i].ptr == 0 ){
			// check how much size is in this heap			
			if ( i == 0){
				MEM2HeapArray[i].ptr = (unsigned int*)MEM2StorPtr;
				MEM2HeapArray[i].size = size;
			}
			else {			
				MEM2HeapArray[i].ptr = (unsigned int*)(MEM2HeapArray[i-1].ptr + (unsigned int)MEM2HeapArray[i-1].size);	
				MEM2HeapArray[i].size = size;
			}
			break;
		}
	}
	if ( i == HEAP_CNT ) return 0;
	else return MEM2HeapArray[i].ptr;
	*/
}

void freeMEM2(void * ptr)
{
	MEM2StorPtr = 0x90000020;
/*
	for(int i = 0; i < HEAP_CNT; i++){
		if ( MEM2HeapArray[i].ptr == ptr){
			MEM2HeapArray[i].ptr = 0;
			MEM2HeapArray[i].size = 0;		
		}	
	}
*/
}

