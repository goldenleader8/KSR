#include <iostream>
#include <cmath>
#include <array>
#include <iomanip>
#include <vector>

const double PHI = 3.141592653589793;
// Link length
const double a1 = 10.30;
const double a2 = 12.441;
const double a3 = 10.688;

// Twist angle
const double alpha1 = 90;
const double alpha2 = 0;
const double alpha3 = 0;

// Offset
const double d1 = 9.860;
const double d2 = 0;
const double d3 = 0;

double theta_deg;
int dof = 3, k;
using Mat4x4 = std::array<std::array<double, 4>, 4>;

Mat4x4 translationMatrix(double theta_deg, double di, double a, double alpha_deg)
{
double alpha = (alpha_deg * PHI) / 180;
double theta = (theta_deg * PHI) / 180;
    Mat4x4 mat = {
      std::array<double, 4> { cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta) },
      std::array<double, 4> { sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta) },
      std::array<double, 4> { 0, sin(alpha), cos(alpha), di },
      std::array<double, 4> { 0, 0, 0, 1 }, 
    };
    return mat;
}

Mat4x4 identity(){
  return {
    std::array<double,4>{1,0,0,0},
    std::array<double,4>{0,1,0,0},
    std::array<double,4>{0,0,1,0},
    std::array<double,4>{0,0,0,1}
  };
}

Mat4x4 multiply(const Mat4x4& A, const Mat4x4& B)
{
    Mat4x4 C{};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

int main()
{
  printf("\n\n\t==[ROBOT ARM BIRU]==");
  printf("\nIngat, robot ini memiliki %d DOF", dof);
  std::vector<Mat4x4> T(dof);
   do
   {
    printf("\n\t==[LINK 1]==");
    printf("\nSudut rotasi joint ke-1: ");scanf("%lf", &theta_deg);
    if (theta_deg > 360 || theta_deg < -360)
    {
        printf("Sudut rotasi tidak valid. Masukkan nilai antara -360 dan 360.\n");
        continue;
    }
   } while (theta_deg > 360 || theta_deg < -360);
    T[0] = translationMatrix(theta_deg, d1, a1, alpha1);
    
   do
   {
    printf("\n\t==[LINK 2]==");
    printf("\nSudut rotasi joint ke-2: ");scanf("%lf", &theta_deg);
    if (theta_deg > 360 || theta_deg < -360)
    {
        printf("Sudut rotasi tidak valid. Masukkan nilai antara -360 dan 360.\n");
        continue;
    }
   } while (theta_deg > 360 || theta_deg < -360);
    T[1] = translationMatrix(theta_deg, d2, a2, alpha2);

   do
   { 
    printf("\n\t==[LINK 3]==");
    printf("\nSudut rotasi joint ke-3: ");scanf("%lf", &theta_deg);
    if (theta_deg > 360 || theta_deg < -360)
    {
        printf("Sudut rotasi tidak valid. Masukkan nilai antara -360 dan 360.\n");
        continue;
    }
   } while (theta_deg > 360 || theta_deg < -360);
    T[2] = translationMatrix(theta_deg, d3, a3, alpha3);

  Mat4x4 T_total = identity();
  std::cout << std::fixed << std::setprecision(4);
    printf("\n\n\t==[MATRIX TRANFORMASI]==\n");
    for (k = 0; k < dof; k++)
      {
        T_total = multiply(T_total ,T[k]);
      }
    for (const auto& row : T_total)
      {
        std :: cout << "| ";
        for (const auto& col : row)
            std::cout << std::setw(8) << col << " ";
        std::cout << "|\n";
      }
  printf("\n\t==[POSISI END-EFFECTOR]==\n");
  printf("X = %.4f\n", T_total[0][3]);
  printf("Y = %.4f\n", T_total[1][3]);
  printf("Z = %.4f\n", T_total[2][3]);
  return 0;
}
