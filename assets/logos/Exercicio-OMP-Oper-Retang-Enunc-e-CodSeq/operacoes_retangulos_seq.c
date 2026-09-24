// compilar com -lm

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// variaveis globais
int *base,*altura;
int *perimetro, *area;
double *diagonal;

void ret_perimetro (int dim)
{
    int i;
    for (i = 0; i < dim; i++)
    {
        perimetro[i] = (base[i]*2) + (altura[i]*2);
    }
}

void ret_area (int dim)
{
    int i;
    for (i = 0; i < dim; i++)
    {
        area[i] = base[i] * altura[i];
    }
}

void ret_diagonal(int dim)
{
    double num;
    int i;
	
    for (i = 0; i < dim; i++){
        num = (pow(base[i],2) + pow(altura[i],2));
        diagonal[i] = sqrt(num);
    }
}

int main()
{
    int i,dim;

    fscanf(stdin, "%d\n", &dim); // Lê a dimensão dos vetores

    // Aloca os vetores
    base=(int *)malloc(dim * sizeof(int));
    altura=(int *)malloc(dim * sizeof(int)); 
	
	perimetro = (int *)malloc(dim * sizeof(int));
	area = (int *)malloc(dim * sizeof(int));
	diagonal =(double *)malloc(dim * sizeof(double));
    
    for(i = 0; i < dim; i++)
	{
        fscanf(stdin, "%d %d ",&(base[i]), &(altura[i])); // Lê os vetores base e altura
	}

    ret_perimetro(dim);
    ret_area(dim);
    ret_diagonal(dim);

    for (i = 0; i < dim; i++)
    {
    	printf("base[%d]=%d, alt[%d]=%d, per[%d]=%d, area[%d]=%d, diag[%d]=%.2f\n", i, base[i], i, altura[i], i, perimetro[i], i, area[i], i, diagonal[i]);
	fflush(0);
    }
    
    return 0;
}
