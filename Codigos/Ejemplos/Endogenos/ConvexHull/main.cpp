#include <iostream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iomanip>
using namespace std;
typedef struct Vectores{
    double i;
    double j;
//    int k;
}vectores_t;

vectores_t crear_vector(){
vectores_t llenado;
 system("cls");
 cout << "Ingrese el valor de X: ";
    cin >> llenado.i;
    cout << "\nIngrese el valor de Y: ";
    cin >> llenado.j;
 //   cout <<"\nIngrese el Valor de Z: ";
   // cin >> llenado.k;
    cout <<"\nSe ha almacenado los datos." << endl;
 system("cls");
return llenado;
}

void mostrar_vectores(vectores_t vec)
{
    cout << "Valor en X: " << vec.i << endl;
    cout << "Valor en Y: " << vec.j << endl;
 //   cout << "Valor en Z: " << vec.k << endl;
     system("pause");

}

int main()
{
    vectores_t A,B,P,AB,AP,BP,BA,puntos[10],izquierda_mayor,recta_a,recta_b,orden[10];
    double a,b,c,d1,d2,d,dmenor,aa,bb,aa1,aa2,bb1,bb2,angulo,primero,segundo1,segundo2;
    bool bandera_1,bandera_2;
        int resultado_a, resultado_b,contador,angulo_menor,direccion,puerta[10],punto;

int selec=0;
 int n;

        double menor=0;

bool opcion=false;



 while (! opcion){
      system("cls");
cout <<"1) Crear Ecuacion de la recta (r)" << endl;
cout <<"2) Encontrar Distancia minnima (P,r)" << endl;
cout <<"3) Encontrar la Distancia minima P vs Seg.Recta (P,PA<->PB)" << endl;
cout <<"4) Crear un Poligono Convexo (ConvexHull) con n Puntos" << endl;
/*
cout <<"3) Crear AB" << endl;
cout <<"3) Crear AC" << endl;
*/
cout <<"5) Salir" << endl;
cin >>selec;
cin.ignore();
if (selec==4)
{
     system("cls");
       cout << "Indique el numero de puntos" << endl;
       cin >>n;
       cin.ignore();

}
        switch(selec){
    case 1:
        system("cls");
        cout << "Llene los valores del punto A: " << endl;
        system("pause");
 A=crear_vector();
 cout << "Llene los valores del punto B: " << endl;
 system("pause");
 B=crear_vector();
a=(B.j-A.j);
b=(A.i-B.i);
c=((-A.i*B.j)+(A.i*A.j)+(A.j*B.i)+(-A.j*A.i));
 cout << "La ecuacion de la recta es: " << "(" <<a << "x" << ")" << "+" << "(" << b<< "y" << ")" << "+" <<"(" << c << ")"<< "=0" << endl;
 system("pause");
 break;
    case 2:
        system("cls");
       cout << "Llene los valores del punto P: " << endl;
        system("pause");
        P=crear_vector();
        d1=a*P.i+b*P.j+c;
        if (d1<0)
        {
            d1=d1*(-1);
        }
        aa=a*a;
        bb=b*b;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d2=sqrt(aa+bb);
        d=d1/d2;
        cout << "La distancia minima entre el Punto P y La Recta r: " << d << endl;
        system("pause");
        break;
    case 3:
    system("cls");
       cout << "Llene los valores del punto P: " << endl;
        system("pause");

        P=crear_vector();
        AB.i=B.i-A.i;
        AB.j=B.j-B.j;
        BA.i=A.i-B.i;
        BA.j=A.j-B.j;
        AP.i=P.i-A.i;
        AP.j=P.j-A.j;
        BP.i=P.i-B.i;
        BP.j=P.j-B.j;
        resultado_a=AB.i*AP.i+AB.j+AP.j;
        resultado_b=BA.i*BP.i+BA.j*BP.j;
        if (resultado_a<0)
        {
            bandera_1=false;
        }
            else
            {


                bandera_1=true;
        }
        if (resultado_b<0)
        {
            bandera_2=false;
        }
            else
            {


                bandera_2=true;
        }
            if (bandera_1==false && bandera_2==true)
            {
                cout << "Sector 1" << endl;
                 aa=AP.i*AP.i;
        bb=AP.j*AP.j;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d=sqrt(aa+bb);
        cout << "La distancia minima (P,A<->B): " << d << endl;


            }
            if (bandera_1==true && bandera_2==true)
            {
                cout << "Sector 2" << endl;
                        d1=a*P.i+b*P.j+c;
        if (d1<0)
        {
            d1=d1*(-1);
        }
        aa=a*a;
        bb=b*b;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d2=sqrt(aa+bb);
        d=d1/d2;
  cout << "La distancia minima (P,A<->B): " << d << endl;

            }
            if (bandera_1==true && bandera_2==false)
            {
                cout<<"Sector 3" << endl;
                aa=BP.i*BP.i;
        bb=BP.j*BP.j;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d=sqrt(aa+bb);
        cout << "La distancia minima (P,A<->B): " << d << endl;
            }
            system("pause");
            break;

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/













/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% CASE 4: ESTA SELECCION ES LA QUE PERMITE EL CALCULO      %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% DEL TRIANGULO CONVEXO, TAMBIEN SE CALCULA LA DISTANCIA   %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% MENOR ENTRE EL PUNTO DE GRAVEDAD Y EL TRIANGULO CONVEXO. %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
    case 4:
        angulo_menor=360;
        dmenor=1000;


        system("pause");


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% LOOPS DESTINADOS PARA CREAR EL TRIANGULO CONVEXO  %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
int entrada;
        for (int p=0; p<n;p++)
        {
            puntos[p]=crear_vector();
        }
        menor=puntos[0].i;
        izquierda_mayor=puntos[0];
        for (int p=0;p<n;p++)
        {
            puerta[p]=1;
           if (menor>puntos[p].i)
           {
               menor=puntos[p].i;
               izquierda_mayor=puntos[p];
               entrada=p;

           }
        }
        cout << "El mayor a la izquiera es: " << menor << endl;
        system("pause");
        recta_a.i=0;
       recta_a.j=(izquierda_mayor.j+1)-izquierda_mayor.j;

orden[0]=izquierda_mayor;
puerta[entrada]=0;
contador=0;

        for (int p=0;p<n;p++)
        {
            angulo_menor=360;
            for (int k=0;k<n;k++)
            {


            if ((puntos[k].i != izquierda_mayor.i or puntos[k].j != izquierda_mayor.j) and puerta[k]!=0)
            {
                recta_b.i=puntos[k].i-izquierda_mayor.i;
                recta_b.j=puntos[k].j-izquierda_mayor.j;
                aa1=recta_a.i;
                aa2=recta_a.j;
                bb1=recta_b.i;
                bb2=recta_b.j;
                if (aa1<0)
                {
            aa1=aa1*-1;
                }
                if (aa2<0)
                {
            aa2=aa2*-1;
                }

                if (bb1<0)
                {
            bb1=bb1*-1;
                }

                if (bb2<0)
                {
            bb2=bb2*-1;
                }

primero=((recta_a.i*recta_b.i)+(recta_a.j*recta_b.j));
segundo1=sqrt((aa1*aa1)+(aa2*aa2));
segundo2=sqrt((bb1*bb1)+(bb2*bb2));
               angulo=(acos(primero/(segundo1*segundo2)))*180/3.14;
               if (izquierda_mayor.i>puntos[k].i)
    {
        angulo=360-angulo;
    }
if (angulo<angulo_menor and (izquierda_mayor.i!=puntos[k].i or izquierda_mayor.j>puntos[k].j))
{
    direccion=k;
    angulo_menor=angulo;

       puerta[entrada]=1;





}


            }
            }

            contador=contador+1;

            orden[contador]=puntos[direccion];
            izquierda_mayor=puntos[direccion];
            puerta[direccion]=0;
            if (puntos[direccion].i==orden[0].i and puntos[direccion].j==orden[0].j)
            {
                    punto=contador+1;
                break;
            }

        }
for (int p=0;p<punto;p++)
{
    cout << "(" << orden[p].i << "," << orden[p].j << ")" << endl;


}
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% CALCULO DE LA DISTACIA MINIMA ENTRE CADA SEGMENTO %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% DE LA RECTA QUE CONFORMA EL TRIANGULO CONVEXO     %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

cout << "Indique el Valor del Punto de Gravedad" << endl;
P=crear_vector();
system("pause");
for (int p=0;p<punto-1;p++)
{
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Se Almacenan los valores de los puntos pertenecientes    %%%%%%%%%%%%%%
%%%%%%%%%%% Al Triangulo Convexo en nuevas Variables, para Efectuar  %%%%%%%%%%%%%%
%%%%%%%%%%% El Calculo de la  Distancia minima entre el punto de     %%%%%%%%%%%%%%
%%%%%%%%%%% Gravedad vs Segmento De la Recta.                        %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
A.i=orden[p].i;
A.j=orden[p].j;
B.i=orden[p+1].i;
B.j=orden[p+1].j;

AB.i=B.i-A.i;
AB.j=B.j-A.j;
BA.i=A.i-B.i;
BA.j=A.j-B.j;
AP.i=P.i-A.i;
AP.j=P.j-A.j;
BP.i=P.i-B.i;
BP.j=P.j-B.j;
resultado_a=AB.i*AP.i+AB.j*AP.j;
resultado_b=BA.i*BP.i+BA.j*BP.j;
a=(B.j-A.j);
b=(A.i-B.i);
c=((-A.i*B.j)+(A.i*A.j)+(A.j*B.i)+(-A.j*A.i));
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

        if (resultado_a<0)
        {
            bandera_1=false;
        }
            else
            {


                bandera_1=true;
        }
        if (resultado_b<0)
        {
            bandera_2=false;
        }
            else
            {


                bandera_2=true;
        }
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Sector 1: El condicional se cumple siempre y cuando el punto %%%%%%%%%%%%
%%%%%%%%% De gravedad No sea perpendicular al segmento de recta y se   %%%%%%%%%%%%
%%%%%%%%% encuentre al extremo izquierdo del plano.                    %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

            if (bandera_1==false && bandera_2==true)
            {
                cout << "Sector 1" << endl;
                 aa=AP.i*AP.i;
        bb=AP.j*AP.j;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d=sqrt(aa+bb);
        cout << "La distancia minima (P,A<->B): " << d << endl;


            }
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Sector 2: El condicional se cumple siempre y cuando el punto %%%%%%%%%%%%%
%%%%%%%% de Gravedad sea perpendicular al segmento de recta.          %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
            if (bandera_1==true && bandera_2==true)
            {
                cout << "Sector 2" << endl;
                        d1=a*P.i+b*P.j+c;
        if (d1<0)
        {
            d1=d1*(-1);
        }
        aa=a*a;
        bb=b*b;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d2=sqrt(aa+bb);
        d=d1/d2;
  cout << "La distancia minima (P,A<->B): " << d << endl;

            }
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Sector 1: El condicional se cumple siempre y cuando el punto %%%%%%%%%%%%
%%%%%%%%% De gravedad No sea perpendicular al segmento de recta y se   %%%%%%%%%%%%
%%%%%%%%% encuentre al extremo derecho del plano.                      %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
            if (bandera_1==true && bandera_2==false)
            {
                cout<<"Sector 3" << endl;
                aa=BP.i*BP.i;
        bb=BP.j*BP.j;
        if (aa<0)
        {
            aa=aa*(-1);
        }
        if (bb<0)
        {
            bb=bb*(-1);
        }
        d=sqrt(aa+bb);
        cout << "La distancia minima (P,A<->B): " << d << endl;
            }
            if (d<dmenor)
            {
                dmenor=d;
            }
            system("pause");
}
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
cout << "La Distancia minima entre el Punto de gravedad y el triangulo convexo es: " << dmenor << endl;
system("pause");
break;
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
















/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/



case 5:
        opcion=true;
        break;
 /*
break;
case 2:
cout <<"1) A" << endl;
cout <<"2) B" << endl;
cout <<"3) C" << endl;
cin >>selec_2;
cin.ignore();
switch(selec_2){
case 1:
    mostrar_vectores(A);
    selec=0;
    break;
    break;
case 2:
    mostrar_vectores(B);
    selec=0;
    break;
    break;

    case 3:
    mostrar_vectores(C);
    selec=0;
    break;
    break;

}
selec=0;
break;

case 3:

    AB.i=B.i-A.i;
    AB.j=B.j-A.j;
    AB.k=B.k-A.k;
break;
case 4:

    AC.i=C.i-A.i;
    AC.j=C.j-A.j;
    AC.k=C.k-A.k;
break;
case 10:
opcion=true;

break;
}
*/
}
        }
        return 0;

}
