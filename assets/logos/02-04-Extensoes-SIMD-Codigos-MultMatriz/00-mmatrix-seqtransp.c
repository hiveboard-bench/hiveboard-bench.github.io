// to compile: gcc 00-mmatrix-seqtransp.c -o 00-mmatrix-seqtransp -Wall
// to run: 00-mmatrix-seqtransp

#include<stdlib.h>
#include<stdio.h>

void transpose(int order, int *matriz) {
  int i, j, aux;
  
  for (i = 0; i < order; i++) 
  {
    for (j = i+1; j < order; j++) 
    {
        if (j != i) 
        {
            aux = matriz[i*order+j];
            matriz[i*order+j] = matriz[j*order+i];
            matriz[j*order+i] = aux;
        } // end if
    } // end for
  } // end for
   
} // end transpose


int main(int argc, char **argv)
{
    int     	*a, *b, *c;
    int		order, i, j, k;
    
//  char 	filename[20]; // used just when debugging
//  FILE 	*trace; // used just when debugging

    // order of our square matrices (A, B and C)
    order = 1000;
   
//  These files are used for tracing purposes. By default, just process 0 has right to print on stdout.
//  Observe the file names. There will be a different file for each process, according to their names.
//  sprintf(filename,"rank_%d.trace", my_rank);
//  trace = fopen(filename, "w");
    
    // creating matrices
    a = (int *) calloc((order*order), sizeof(int));
    b = (int *) calloc((order*order), sizeof(int));
    c = (int *) calloc((order*order), sizeof(int));

    // filling up them
    k = 1;
    for(i = 0; i < order; i++) 
    {
        for(j=0; j < order; j++)  
        {
          a[i*order+j] = b[i*order+j] = k;
        } // end-for-j
        k++;
    }// end-for-i

    // calcula a transposta de b
    transpose(order, b);
    
    for (i = 0; i < order; i++)
    {
        for (j = 0; j < order; j++)
        {
            c[i*order+j]=0;
            for(k = 0; k < order; k++)
            {
    //		evaluates taking into account the transpose of b
            c[i*order+j] += a[i*order+k]*b[j*order+k]; 
            }
        }
    }

/*  for(i = 0; i < order; i++) 
    {
        for(j=0; j < order; j++)  
        {
            printf("c[%d][%d]= %d \n", i, j, c[i*order+j]);
            fflush(0);
        }//end-for-j
    }//end-for-i
*/  
    exit(0);
    
}// end-main
