#include<stdio.h>
#include<stdlib.h>
#include<math.h>

int menu;
const float PHI = 3.14159263;
float x, y, z;
const float a1 = 9.087, a2 = 12.441, a3 = 10.688;
float r1, r2, r3;
float phi1, phi2, phi3;
float tetha1, tetha2, tetha3;
float deg1, deg2, deg3;

void IK(float *tetha1, float *tetha2, float *tetha3, float x, float y, float z);

float clamp(float val, float min, float max){ //acos hanya berfungsi terhadap -1 <= x <= 1. Misal, jika acos(-5) = NaN
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

int main(){
    do
    {
        printf("\n\n\t==[Inverse Kinematics]==");
        printf("\n[1] Inverse Kinematics Robot 3 DOF");
        printf("\n[0] Exit Program");
        printf("\nPilih Opsi Menu: ");scanf("%d", &menu);
        switch (menu)
        {
        case 1:
            printf("\n\n\t==[Inverse Kinematics]==");
            printf("\nMasukkan koordinat X: ");scanf("%f", &x);
            printf("\nMasukkan koordinat Y: ");scanf("%f", &y);
            printf("\nMasukkan koordinat Z: ");scanf("%f", &z);

            IK(&tetha1, &tetha2, &tetha3, x, y, z);

            deg1 = (tetha1 * 180)/PHI;
            deg2 = (tetha2 * 180)/PHI;
            deg3 = (tetha3 * 180)/PHI;

            printf("\n\n\t==[Pergerakan tiap Servo]==");
            printf("\n Target End Effector (XYZ): (%.2f, %.2f, %.2f)", x, y, z);
            printf("\n Base rotation sebesar: %.2f°", deg1);
            printf("\n Joint 1 berputar sebesar: %.2f°", deg2);
            printf("\n Joint 2 berputar sebesar: %.2f°", deg3);
            break;

        case 0:
            printf("\n Exiting Program....");
            break;

        default:
            printf("\n[!] Tidak ada opsi [!]");
            break;
        }
    } while (menu != 0);
}

void IK(float *tetha1, float *tetha2, float *tetha3, float x, float y, float z){
    *tetha1 = atan2(y, x);
    r1 = sqrt((x*x)+(y*y));
    r2 = z - a1;
    r3 = sqrt((r2*r2)+(r1*r1));
    phi2 = atan2(r2, r1);
    phi1 = acos(clamp(((a2*a2)+(r3*r3)-(a3*a3))/(2*a2*r3), -1.0f, 1.0f));
    phi3 = acos(clamp(((a2*a2)+(a3*a3)-(r3*r3))/(2*a2*a3), -1.0f, 1.0f));
    *tetha2 = phi2 - phi1;
    *tetha3 = PHI - phi3;
}