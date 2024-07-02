// Physics Engine.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "Physics Engine.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <objidl.h>
#include <gdiplus.h>
#include <gdiplusgraphics.h>
#include <chrono>
#include <Eigen/Dense>
#include <map>


using Eigen::MatrixXd;
;
using Eigen::VectorXd;

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // TODO: Place code here.

    // Initialize global strings
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_PHYSICSENGINE, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // Perform application initialization:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_PHYSICSENGINE));

    MSG msg;

    // Main message loop:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int) msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_PHYSICSENGINE));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_PHYSICSENGINE);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

void printMatrix(MatrixXd* matpoint)
{
    MatrixXd mat = *matpoint;
    std::stringstream ss;
    ss << mat;
    std::string debugMat = ss.str();
    std::wstring ws = std::wstring(debugMat.begin(), debugMat.end());
    OutputDebugStringA("\n------\n");
    OutputDebugStringW(ws.c_str());
}

void printVector(VectorXd* vecpoint)
{
    VectorXd vec = *vecpoint;
    std::stringstream ss;
    ss << vec;
    std::string debugVec = ss.str();
    std::wstring ws = std::wstring(debugVec.begin(), debugVec.end());
    OutputDebugStringA("\n------\n");
    OutputDebugStringW(ws.c_str());
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
typedef struct vector_struct {
    double x = 0;
    double y = 0;
    double z = 0;

    void operator +=(const vector_struct& rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;
    }

    double magnitude()
    {
        return sqrt(x * x + y * y + z * z);
    }


    double mag()
    {
        return magnitude();
    }

    vector_struct normalized()
    {
        double m = magnitude();
        return struct vector_struct({ x / m, y / m, z / m });
    }
    
} Vector;

Vector& operator +(const Vector& a, const Vector& b)
{
    Vector c = { a.x + b.x, a.y + b.y, a.z + b.z };
    return c;
}



Vector& operator *(const Vector& a, const double& b)
{
    Vector c = { a.x*b, a.y*b, a.z*b };
    return c;
}

Vector& operator *(const double& a, const Vector& b)
{
    Vector c = b*(a);
    return c;
}

Vector& operator -(const Vector& a, const Vector& b)
{
    Vector nb = b * (-1);
    Vector c = a + nb;
    return c;
}

Vector cross(Vector a, Vector b)
{
    Vector c = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
    return c;
}

Vector rotation2D(Vector v, double angle)
{
    double currentAngle = atan2(v.y, v.x);
    double currentDistance = sqrt(v.x * v.x + v.y * v.y);
    Vector newVector = { cos(currentAngle + angle), sin(currentAngle + angle) };
    newVector = currentDistance*newVector;
    return newVector;
}

Vector& operator *(const Vector& a, const Vector& b)
{
    Vector c = cross(a, b);
    return c;
}

double dot(Vector a, Vector b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector averageVector(Vector* vectors, int arrLength)
{
    Vector base = { 0,0,0 };
    int count = arrLength;
    for (int i = 0; i < arrLength; i++)
    {
        base += vectors[i];
    }
    base = base * (1.0 / (double)count);
    return base;
}


typedef struct box_struct
{
    Vector position;
    Vector rotation;
    Vector velocity = { 0,0,0 };
    Vector rotationalVelocity = { 0,0,0 };
    Vector acceleration = { 0,0,0 };
    Vector rotationalAcceleration = { 0,0,0 };
    Vector netForce = { 0, 0, 0 };
    Vector netMoment = { 0, 0, 0 };
    Vector center = {NULL, NULL, NULL};
    double MOI = 1.00;
    double mass = 1.0;

    Vector* bodyPoints;
    int bodyPointsLength;

    void addPoints(std::vector<Vector> points)
    {
        bodyPointsLength = points.size();
        bodyPoints = (Vector*)malloc(sizeof(Vector) * bodyPointsLength);
        for (int i = 0; i < bodyPointsLength; i++)
        { 
            bodyPoints[i] = points[i];
        }

        center = averageVector(bodyPoints, bodyPointsLength);
        Vector cen = center;
        position += center;
        for (int i = 0; i < bodyPointsLength; i++)
        {
            Vector negativeCenter = (-1) * center;
            bodyPoints[i] += negativeCenter;
        }
    }
} Box;

typedef struct modular_term
{
    double coefficient;
    int variableIndex;
} MT;

typedef struct linear_term: modular_term
{
    double coefficient;
    std::string variableName;

    MT toMT(std::map<std::string, int> varMap)
    {
        return MT({ coefficient, varMap[variableName] });
    }
} LT;



typedef struct dynamic_system
{
    MatrixXd coefficients;
    VectorXd solutions;
    VectorXd solutionCache;
    bool updatedCache = false;
    Box* body;
    std::map<std::string, int> variableMap;

    int variableCount = 0;
    int equationCount = 0;


    dynamic_system(Box* connectedBody) //Ax and Ay
    {
        body = connectedBody;
        int initialSize = 3;
        coefficients = Eigen::MatrixXd::Zero(initialSize, initialSize);
        solutions = Eigen::VectorXd::Zero(initialSize);
        printMatrix(&coefficients);
        variableCount = 3;
        equationCount = 3;

        coefficients(0, 0) = -body->mass;
        coefficients(1, 1) = -body->mass;
        coefficients(2, 2) = -body->MOI;
        

        //Equation 1: (-m)Ax = 0
        //Equation 2: (-m)Ay = 0
        //Equation 3: M = 0
    }

    void expand(int equationCount)
    {
        while (equationCount > coefficients.rows())
        {
            coefficients.resize(coefficients.rows() + 1, coefficients.cols());
        }
    }


    void addVariable(Vector direction, Vector appliedAt, std::string variableName = "NULL")
    {
        updatedCache = false;
        int variableIndex = variableCount;
        int equationIndex = variableCount;
        variableCount++;
        
        while (variableCount > coefficients.cols())
        {
            int newRowSize = coefficients.rows() + 1;
            int newColSize = coefficients.cols() + 1;
            coefficients.conservativeResize(newRowSize, newColSize);
            solutions.conservativeResize(newRowSize);
            solutions(newRowSize - 1) = 0;
            for (int i = 0; i < newRowSize; i++)
            {
                coefficients(newColSize - 1, i) = 0;
                coefficients(i, newRowSize - 1) = 0;
            }
            
        }

        Vector r = appliedAt - body->position;
        
        double xCoeff = direction.x;
        double yCoeff = direction.y;

        double xMomentCoeff = -r.y*xCoeff; //AxBy - AyBx
        double yMomentCoeff = r.x*yCoeff;

        //Add to FNetX eq
        coefficients(0, variableIndex) = xCoeff;
        //Add to FNetY eq
        coefficients(1, variableIndex) = yCoeff;
        //Add to Moment eq
        coefficients(2, variableIndex) = xMomentCoeff + yMomentCoeff;

        printMatrix(&coefficients);
        //Add name to variable dictionary
        if (variableName._Equal("NULL")) { return; }

        variableMap.insert({ variableName,variableIndex });
        
    }

    void registerNewEquation()
    {
        if (equationCount > coefficients.rows())
        {
            equationCount++;
        }
    }

    void setConstant(int variableIndex, double value)
    {
        expand(equationCount + 1);

        coefficients(equationCount, variableIndex) = 1;
        solutions(equationCount) = value;

        equationCount++;
    }

    void setConstantWithName(std::string variableName, double value)
    {
        int variableIndex = variableMap[variableName];
        setConstant(variableIndex, value);
    }

    int getIndex(std::string variableName)
    {
        return variableMap[variableName];
    }

    void addEquation(std::vector<MT> terms, double solution = 0)
    {
        
        if (equationCount >= variableCount) { return; }

        int equationIndex = equationCount;
        for (int i = 0; i < terms.size(); i++)
        {
            MT term = terms[i];
            coefficients(equationIndex, term.variableIndex) = term.coefficient;
        }
        solutions(equationIndex) = solution;
        equationCount++;
    }

    void addEquation(std::vector<LT> terms, double solution = 0)
    {
        std::vector<MT> MTterms;

        for (int i = 0; i < terms.size(); i++)
        {
            MTterms.push_back(MT({ terms[i].coefficient, getIndex(terms[i].variableName) }));
        }

        addEquation(MTterms, solution);
    }

    VectorXd solve()
    {
        if (updatedCache)
        {
            return solutionCache;
        }
        updatedCache = true;

        VectorXd variableValues = coefficients.lu().solve(solutions);
        printVector(&variableValues);
        printMatrix(&coefficients);
        printVector(&solutions);
        
        solutionCache = variableValues;
        return variableValues;
    }

    Vector solveAcceleration()
    {
        VectorXd varVals = solve();
        return Vector({ varVals(0), varVals(1), 0});
    }

    Vector solveAngularAcceleration()
    {
        VectorXd varVals = solve();
        double rot = varVals(2);
        return Vector({ 0, 0, varVals(2)});
    }



    
} DynamicSystem;

const Vector GRAVITY({ 0, 98.1,0 });

void applyForce(Box* b, Vector force, Vector appliedAt)
{
    b->netForce += force;
    Vector center = b->position;
    Vector r = appliedAt - center;
    Vector appliedMoment = cross(r, force);
    b->netMoment += appliedMoment;
}

void applyMoment(Box* b, Vector moment)
{
    b->netMoment += moment;
}



void processBox(Box* b, double deltaTime, RECT windowRect)
{

    int width = windowRect.right - windowRect.left;
    int height = windowRect.bottom - windowRect.top;
    ;
    if (b->center.x == NULL)
    {
        b->center = averageVector(b->bodyPoints, b->bodyPointsLength);
        Vector cen = b->center;
        b->position += b->center;
        for (int i = 0; i < b->bodyPointsLength; i++)
        {
            Vector negativeCenter = (-1) * b->center;
            b->bodyPoints[i] += negativeCenter;
        }
    }
    
    DynamicSystem ds(b);

    ds.addVariable(Vector({ 0, 1 }), b->position); //gravity: varIndex = 3;
    ds.setConstant((int)3, GRAVITY.y);

    for (int i = 0; i < b->bodyPointsLength; i++)
    {

        Vector localPoint = b->bodyPoints[i];
        localPoint = rotation2D(localPoint, b->rotation.z);
        Vector globalPoint = localPoint + b->position;

        if (globalPoint.y > height - 100)
        {
            //a.y * b.z - a.z * b.y,
            //a.z* b.x - a.x * b.z,
            //a.x * b.y - a.y * b.x
            //Ab = Aa + alpha x r(b/a) - omega^2 * r(b/a)
            ds.addVariable(Vector({ 0, -1 }), globalPoint); //id: 3 + i    
            int varID = 3 + i;
            double omega = b->rotationalVelocity.mag();
            double sol = (omega * omega) * localPoint.y;

            Vector relativeVelocity = cross(b->rotationalVelocity, localPoint);
            Vector currentVelocity = b->velocity + relativeVelocity;
            currentVelocity = currentVelocity * (-1);
            
            Vector requiredAverageAcceleration = currentVelocity*(0.5/deltaTime);
            //Ay + (alphaZ*rX) - omega^2 x rY = average force from impulse to not go through collider in deltaTime 
            //Ay + (rX)aZ = Favg + o^2 * rY
            std::vector<MT> terms = { {1, 1}, {localPoint.x, 2} };
            ds.addEquation(terms, sol + requiredAverageAcceleration.y);
            printMatrix(&ds.coefficients);
            Vector corrective = { 0, (height - 100) - globalPoint.y, 0};
            b->position += corrective;
           
        }

    }

    b->acceleration = ds.solveAcceleration();
    b->rotationalAcceleration = ds.solveAngularAcceleration();

    Vector deltaVel = (b->acceleration) * deltaTime;
    b->velocity += deltaVel;
    Vector deltaRotVel = (b->rotationalAcceleration) * deltaTime;
    b->rotationalVelocity += deltaRotVel;

    Vector deltaPos = b->velocity * deltaTime;
    b->position += deltaPos;
    Vector deltaRot = b->rotationalVelocity * deltaTime;
    b->rotation += deltaRot;

}

void processBoxKinematics(Box* b, double deltaTime, RECT windowRect)
{
    
    int width = windowRect.right - windowRect.left;
    int height = windowRect.bottom - windowRect.top;
    ;
    if (b->center.x == NULL)
    {
        b->center = averageVector(b->bodyPoints, b->bodyPointsLength);
        Vector cen = b->center;
        b->position += b->center;
        for (int i = 0; i < b->bodyPointsLength; i++)
        {
            Vector negativeCenter = (-1) * b->center;
            b->bodyPoints[i] += negativeCenter;
        }
    }
    //3 (position: P) + 3(rotation: R) + 3(velocity: V) + 3(angular velocity: O) + 3*number_of_points
    //0: Px
    //1: Py
    //2: Pz
    //3: Rx
    //4: Ry
    //5: Rz
    //6: Vx
    //7: Vy
    //8: Vz
    //9: Ox
    //10: Oy
    //11: Oz
    //12: solution
    
    int variableCount = 2 + 2 * b->bodyPointsLength + 1; //Vx, Vy, P1x, P1y... Pnx, Pny, Oz (we know Pny based on the direction of Pn)
    int equationCount = variableCount;

    equationCount = min(equationCount, variableCount);

    MatrixXd coefficients = MatrixXd::Zero(equationCount, variableCount);
    
    VectorXd solutions(equationCount);

    

    Vector deltaVel = GRAVITY * deltaTime;
    Vector newVel = b->velocity + deltaVel;
    coefficients(0, 0) = 1;
    solutions(0) = newVel.x;
    coefficients(1, 1) = 1;
    solutions(1) = newVel.y;
    
    /*Vector gravitationalForce = b->mass * GRAVITY;
    Vector predictivePosition = b->position;
    Vector predictiveRotation = b->rotation;
    Vector predictiveVelocity = b->velocity;
    Vector predictiveAngularVelocity = b->rotationalVelocity;*/

    //Vx,Vy,Vz,Ox,Oy,Oz
    int subEquationCount = 3;
    int lowestIndex = 2 - subEquationCount;
    for (int i = 0; i < b->bodyPointsLength; i++)
    {
        //v(p/c) = omega(p/c) x distance(p/c) (d is a coefficient)
        // 
        //expand:
        // 
        //Vp = Vc + (-O) x D
        // 
        //expand:
        // 
        //Vpx = Vcx + (-Oy)Dz - (-Oz)Dy
        //Vpy = Vcy + (-Oz)Dx - (-Ox)Dz
        // 
        //rearrange:
        // 
        //Vcx = Vpx + OyDz - OzDy
        //Vcy = Vpy + OzDx + OxDz 
        // 
        //assume Ox = 0, Oy = 0:
        // 
        //Vcx = Vpx - OzDy
        //Vcy = Vpy + OzDx
        //
        //rearrange:
        //
        //Vcx + (-1)Vpx + (+Dy)Oz = 0
        //Vcy + (-1)Vpy + (-Dx)Oz = 0
        //
        //(Vpx - Vcx) = (Dy/Dx)(Vpy - Vcy)  
        //if [Dx != 0]: (-1)Vcx + (1)Vpx + (-1)(Dy/Dx)Vpy + (Dy/Dx)Vcy = 0 
        //if [Dx  = 0]: (-1)Vcy + (1)Vpy = 0 

        

        
        
        
        lowestIndex += subEquationCount;
        if (i > 0) { subEquationCount = 2; }
        if (lowestIndex >= equationCount) { continue; }
        
        Vector localPoint = b->bodyPoints[i];
        localPoint = rotation2D(localPoint, b->rotation.z);
        Vector globalPoint = localPoint + b->position;

        double totalDistance = localPoint.magnitude();
        
        double directionalAngle = atan2(localPoint.x, localPoint.y);

        double pointCoefficient[] = {-1, -1};
        double omegaCoefficient[] = { localPoint.y, -localPoint.x };

        if (globalPoint.y > height - 100)
        {
            pointCoefficient[0] = 0;
            pointCoefficient[1] = 0;
        }

        for (int ind = 0; ind < 2; ind++)
        {
            int currentIndex = lowestIndex + ind;
            int maxIndex = equationCount;
            if (lowestIndex + ind >= equationCount) { continue; }

            coefficients(lowestIndex + ind, ind) = 1;
            coefficients(lowestIndex + ind, 2 + 2*i + ind) = pointCoefficient[ind];
            coefficients(lowestIndex + ind, variableCount - 1) = omegaCoefficient[ind];
            double pointX = localPoint.x;
            double omegaCoeff = omegaCoefficient[ind];
            solutions(lowestIndex + ind) = 0;
        }

        if (lowestIndex + 2 >= equationCount || i > 0) { continue; }
        //if [Dx != 0]: (-1)Vcx + (1)Vpx + (-1)(Dy/Dx)Vpy + (Dy/Dx)Vcy = 0 
        //if [Dx  = 0]: (-1)Vcy + (1)Vpy = 0 
        if (localPoint.x == 0)
        {
            coefficients(lowestIndex + 2, 1) = -1;
            coefficients(lowestIndex + 2, 2 + 2*i + 1) = 1;
            solutions(lowestIndex + 2) = 0;
        }
        else
        {
            coefficients(lowestIndex + 2, 0) = -1;
            coefficients(lowestIndex + 2, 2 + 2 * i) = 1;

            coefficients(lowestIndex + 2, 1) = -1 * (localPoint.y / localPoint.x);
            coefficients(lowestIndex + 2, 2 + 2 * i + 1) = (localPoint.y/localPoint.x);

            solutions(lowestIndex + 2) = 0;
        }
        
    }
    int endingIndex = equationCount - 3;
    //Final equations for Vx, Vy and Oz

    //Oz equation


    VectorXd variableValues = coefficients.lu().solve(solutions);

    printMatrix(&coefficients);
    printVector(&solutions);
    printVector(&variableValues);
    

    for (int row = 0; row < variableCount; row++)
    {
        for (int col = 0; col < variableCount; col++)
        {
            double foundCoeff = coefficients(row, col);
            if (foundCoeff == 0) { continue; }
            double currentRow = row;
            double currentCol = col;
            int dummy = 0;
        }
        //hi
    }

    double vx = variableValues(0);
    double vy = variableValues(1);
    
    double oz = variableValues(variableCount - 1);

    b->velocity = Vector({ vx, vy, 0 });
    b->rotationalVelocity = Vector({ 0, 0, oz });

    Vector deltaPos = b->velocity * deltaTime;
    b->position += deltaPos;
    Vector deltaRot = b->rotationalVelocity * deltaTime;
    b->rotation += deltaRot;
    
}

//void CALLBACK windowMain(HWND hWnd, UINT uMsg, UINT idEvent, DWORD dwTime);
Box* objects[1] = {};
int objectsLength = 1;
Gdiplus::GdiplusStartupInput gdiplusSI;
ULONG_PTR gdiplusToken;


std::vector<Vector> OpointList = { {0, 0, 0}, {50, 50, 0}, {100, -50, 0}, {130, -75, 0}, {0, -100, 0} };

Box* createBody(std::vector<Vector> pL)
{
    Box* newBox = (Box*)malloc(sizeof(Box));
    (*newBox).position = Vector({ 500, 200, 0 });
    (*newBox).rotation = Vector({ 0, 0, 30 });
    (*newBox).velocity = Vector({ 0, -10, 0 });
    (*newBox).rotationalVelocity = Vector({ 0, 0, 1 });
    (*newBox).addPoints(pL);
    newBox->acceleration = Vector({ 0,0,0 });
    newBox->rotationalAcceleration = Vector({ 0,0,0 });
    newBox->mass = 1;
    newBox->MOI = 1;
    newBox->netForce = Vector({ 0,0,0 });
    newBox->netMoment = Vector({ 0,0,0 });
    return newBox;
}


BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // Store instance handle in our global variable
   int sides = OpointList.size();
   std::vector<Vector> RoundpointList;
   for (int i = 0; i < sides; i++)
   {
       const double pi = 3.1415926535;
       double angle = 2.0 * pi * i / sides;
       RoundpointList.push_back(OpointList[i]);
   }
   
   objects[0] = createBody(RoundpointList);
   
   Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusSI, nullptr);
   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);
   SetMenu(hWnd, NULL);
   

   

   //SetTimer(hWnd,             // handle to main window 
   //    0,            // timer identifier 
   //    (int)(1000),                 // fps interval 
   //    (TIMERPROC)&windowMain);     // no timer callback 

   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//



Vector rotateAroundAxis(Vector vec, Vector axis, double angle)
{
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);
    Vector rowOne = {
        cosAngle + axis.x * axis.x * (1 - cosAngle),
        axis.x * axis.y * (1 - cosAngle) - axis.z * sinAngle,
        axis.x * axis.z * (1 - cosAngle) + axis.y * sinAngle
    };
    Vector rowTwo = {
        axis.y * axis.z * (1 - cosAngle) + axis.z * sinAngle,
        cosAngle + axis.y * axis.y * (1 - cosAngle),
        axis.y * axis.z * (1 - cosAngle) - axis.x * sinAngle
    };
    Vector rowThree = {
        axis.z * axis.x * (1 - cosAngle) - axis.y * sinAngle,
        axis.z * axis.y * (1 - cosAngle) + axis.x * sinAngle,
        cosAngle + axis.z * axis.z * (1 - cosAngle)
    };
    Vector newVec = { dot(rowOne, vec), dot(rowTwo, vec), dot(rowThree, vec) };
    return newVec;
}

double fps = 1;
std::chrono::steady_clock::time_point lastFrame = std::chrono::steady_clock::now();

int windowMain(HWND hWnd)
{
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    long microtimediff = std::chrono::duration_cast<std::chrono::microseconds>(end - lastFrame).count();
    double timediff = (double)microtimediff / (double)(1000000);
    fps = 1.0 / (timediff);
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hWnd, &ps);
    Gdiplus::Graphics gf(hdc);
    Gdiplus::Pen pen(Gdiplus::Color(255, 150, 150, 0));
    RECT windowRect;
    GetWindowRect(hWnd, &windowRect);
    
    int width = windowRect.right - windowRect.left;
    int height = windowRect.bottom - windowRect.top;
    Gdiplus::Bitmap renderMap(width, height, &gf);

    
    
    Gdiplus::Rect winRect(0,0,width,height);

    Gdiplus::Graphics* gmem = Gdiplus::Graphics::FromImage(&renderMap);
    Gdiplus::SolidBrush brush_tr(Gdiplus::Color::White);
    Gdiplus::SolidBrush boxBrush(Gdiplus::Color::Green);
    Gdiplus::Pen boxPen(Gdiplus::Color::DarkGreen,5.0);
    Gdiplus::SolidBrush redBrush(Gdiplus::Color::DarkRed);
    gmem->Clear(Gdiplus::Color::White);

    
    //gf.DrawRectangle(&pen, winRect);
    // TODO: Add any drawing code that uses hdc here...
    for (int i = 0; i < (int)objectsLength; i++)
    {
        Box* box = objects[i];
        
        processBox(box, timediff, windowRect);
        
        double angle = (box->rotation.z);


        Gdiplus::Matrix mat(
            (Gdiplus::REAL)cos(angle),
            (Gdiplus::REAL)sin(angle),
            (Gdiplus::REAL)-sin(angle),
            (Gdiplus::REAL)cos(angle),
            0,
            0
        );
        Gdiplus::Matrix mat2(
            1,
            0,
            0,
            1,
            box->position.x,
            box->position.y
        );
        
        mat.Multiply(&mat2,Gdiplus::MatrixOrderAppend);
        gmem->SetTransform(&mat);
        const int pointCount = box->bodyPointsLength;

        Gdiplus::Point* points = (Gdiplus::Point*)malloc(sizeof(Gdiplus::Point)*pointCount);

        for (int p = 0; p < pointCount; p++)
        {
            Vector pointVec = (box->bodyPoints)[p];
            points[p] = Gdiplus::Point(pointVec.x, pointVec.y);

        }

        

        gmem->FillPolygon(&boxBrush, points, pointCount);
        gmem->DrawPolygon(&boxPen, points, pointCount);
        gmem->FillEllipse(&redBrush, (int)-5,(int)-5,10,10);
        
        for (int p = 0; p < pointCount; p++)
        {
            Vector pi = (box->bodyPoints)[p];
            //gmem->FillEllipse(&redBrush, (int)pi.x-5, (int)pi.y-5, 10, 10);
        }

        gmem->ResetTransform();

        for (int p = 0; p < pointCount; p++)
        {
            Vector pi = (box->bodyPoints)[p];
            pi = rotation2D(pi, angle);
            pi += box->position;
            //gmem->FillEllipse(&redBrush, (int)pi.x - 5, (int)pi.y - 5, 10, 10);
        }

        
        

        free(points);
        

        //FillRect(hdc, &rect, brush);

        

        
    }

    gmem->DrawLine(&pen, 0, (int)height - 100, width, height - 100);
    
    gf.DrawImage(&renderMap, 0, 0);
    //gf.Clear(Gdiplus::Color(255, 0, 0, 0));
    EndPaint(hWnd, &ps);
    lastFrame = end;
    return 0;
}



LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_CREATE:
        SetTimer(hWnd, 1, (int)(1000*1/120.0), NULL);
        break;
    case WM_TIMER:
        InvalidateRect(hWnd, NULL, FALSE);
        break;
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // Parse the menu selections:
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_PAINT:
        {
            //Vector delta = { 0, 0, 0 };
            //Vector newVec = objects[0]->position + delta;
            //objects[0]->position = newVec;
            int objSize = objectsLength;
            int x = 0;
            windowMain(hWnd);
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        Gdiplus::GdiplusShutdown(gdiplusToken);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
