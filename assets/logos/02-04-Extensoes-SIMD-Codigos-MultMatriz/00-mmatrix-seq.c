// to compile: gcc 00-mmatrix-seq.c -o 00-mmatrix-seq
// to run: 00-mmatrix-seq


#include<stdlib.h>
#include<stdio.h>

int main(int argc, char **argv)
{
    int     	*a, *b, *c;
    int		order, i, j, k;
    
//  char 	filename[20]; // used just when debugging
//  FILE 	*trace; // used just when debugging
    
    // order of our square matrices (A, B and C)
    order = 1000;
   
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

    for (i = 0; i < order; i++)
    {
        for (j = 0; j < order; j++)
        {
            c[i*order+j]=0;
            for(k = 0; k < order; k++)
            {
//	     	evaluates directly, without the transpose of b
            c[i*order+j] += a[i*order+k]*b[k*order+j]; 
            }
        }
    }

/*   for(i = 0; i < order; i++) 
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
