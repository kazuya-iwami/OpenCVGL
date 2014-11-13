#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <OpenGL/gl.h>
#include <GLUT/glut.h>

#define WINDOW_X (500)
#define WINDOW_Y (500)
#define WINDOW_NAME "test2"

void init_GL(int argc, char *argv[]);
void init();
void set_callback_functions();

void glut_display();
void glut_keyboard(unsigned char key, int x, int y);
void glut_mouse(int button, int state, int x, int y);
void glut_motion(int x, int y);

void draw_cube();
void draw_pyramid();

// グローバル変数
double g_angle1 = 0.0;
double g_angle2 = 0.0;
double g_distance = 10.0;
bool g_isLeftButtonOn = false;
bool g_isRightButtonOn = false;

int main(int argc, char *argv[]){
    /* OpenGLの初期化 */
    init_GL(argc,argv);
    
    /* このプログラム特有の初期化 */
    init();
    
    /* コールバック関数の登録 */
    set_callback_functions();
    
    /* メインループ */
    glutMainLoop();
    
    return 0;
}

void init_GL(int argc, char *argv[]){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA|GLUT_DEPTH|GLUT_DOUBLE);
    glutInitWindowSize(WINDOW_X,WINDOW_Y);
    glutCreateWindow(WINDOW_NAME);
}

void init(){
    glClearColor(0.0, 0.0, 0.0, 0.0);         // 背景の塗りつぶし色を指定
}

void set_callback_functions(){
    glutDisplayFunc(glut_display);
    glutKeyboardFunc(glut_keyboard);
    glutMouseFunc(glut_mouse);
    glutMotionFunc(glut_motion);
    glutPassiveMotionFunc(glut_motion);
}

void glut_keyboard(unsigned char key, int x, int y){
    switch(key){
        case 'q':
        case 'Q':
        case '\033':
            exit(0);
    }
    
    glutPostRedisplay();
}

void glut_mouse(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON){
        if(state == GLUT_UP){
            g_isLeftButtonOn = false;
        }else if(state == GLUT_DOWN){
            g_isLeftButtonOn = true;
        }
    }
    
    if(button == GLUT_RIGHT_BUTTON){
        if(state== GLUT_UP){
            g_isRightButtonOn = false;
        }else if(state == GLUT_DOWN){
            g_isRightButtonOn = true;
        }
    }
}

void glut_motion(int x, int y){
    static int px = -1, py = -1;
    if(g_isLeftButtonOn == true){
        if(px >= 0 && py >= 0){
            g_angle1 += (double)-(x - px)/20;
            g_angle2 += (double)(y-py)/20;
        }
        px = x;
        py = y;
    }else if(g_isRightButtonOn == true){
        if(px >= 0 && py >= 0){
            g_distance += (double)(y-py)/20;
        }
        px = x;
        py = y;
    }else{
        px = -1;
        py = -1;
    }
    glutPostRedisplay();
}

void glut_display(){
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, 1.0, 0.1, 100);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(g_distance * cos(0)*sin(0), g_distance * sin(0), g_distance * cos(0)*cos(0), 0.0, 0.0, 0.0, 00, 1.0, 0.0);
    //gluLookAt(g_distance * cos(g_angle2)*sin(g_angle1), g_distance * sin(g_angle2), g_distance * cos(g_angle2)*cos(g_angle1), 0.0, 0.0, 0.0, 00, 1.0, 0.0);
    glRotatef(g_angle1*6, 0.0, 0.0, 1.0);
    glRotatef(g_angle2*6, 1.0, 0.0, 0.0);
    
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);//隠れて見えないはずの面も絵画される
    //draw_pyramid();
    draw_cube();
    glFlush();
    glDisable(GL_DEPTH_TEST);
    glutSwapBuffers();
    
}


void draw_pyramid(){
    GLdouble pointO[] = {0.0, 1.0, 0.0};
    GLdouble pointA[] = {1.5, -1.0, 1.5};
    GLdouble pointB[] = {-1.5, -1.0, 1.5};
    GLdouble pointC[] = {-1.5, -1.0, -1.5};
    GLdouble pointD[] = {1.5, -1.0, -1.5};
    
    glColor3d(1.0, 0.0, 0.0);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointA);
    glVertex3dv(pointB);
    glEnd();
    
    

    
    
    glColor3d(1.0, 1.0, 0.0);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointB);
    glVertex3dv(pointC);
    glEnd();
    
    glColor3d(0.0, 1.0, 1.0);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointC);
    glVertex3dv(pointD);
    glEnd();

    
    glColor3d(1.0, 0.0, 1.0);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointD);
    glVertex3dv(pointA);
    glEnd();

    
    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_POLYGON);
    glVertex3dv(pointA);
    glVertex3dv(pointB);
    glVertex3dv(pointC);
    glVertex3dv(pointD);
    glEnd();
}

void draw_cube(){
    GLdouble pointA[] = {1.0, 1.0, 1.0};
    GLdouble pointB[] = {-1.0, 1.0, 1.0};
    GLdouble pointC[] = {-1.0,-1.0, 1.0};
    GLdouble pointD[] = {1.0, -1.0, 1.0};
    GLdouble pointE[] = {1.0, 1.0, -1.0};
    GLdouble pointF[] = {-1.0, 1.0, -1.0};
    GLdouble pointG[] = {-1.0, -1.0, -1.0};
    GLdouble pointH[] = {1.0, -1.0, -1.0};

    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_POLYGON);
    glVertex3dv(pointA);
    glVertex3dv(pointB);
    glVertex3dv(pointC);
    glVertex3dv(pointD);
    glEnd();
    
    glColor3d(0.0, 1.0, 0.6);
    glBegin(GL_POLYGON);
    glVertex3dv(pointB);
    glVertex3dv(pointF);
    glVertex3dv(pointG);
    glVertex3dv(pointC);
    glEnd();
    
    glColor3d(1.0, 0.3, 0.3);
    glBegin(GL_POLYGON);
    glVertex3dv(pointA);
    glVertex3dv(pointB);
    glVertex3dv(pointF);
    glVertex3dv(pointE);
    glEnd();
    
    glColor3d(1.0, 0.0, 1.0);
    glBegin(GL_POLYGON);
    glVertex3dv(pointA);
    glVertex3dv(pointE);
    glVertex3dv(pointH);
    glVertex3dv(pointD);
    glEnd();
    
    
    glColor3d(0.4, 1.0, 1.0);
    glBegin(GL_POLYGON);
    glVertex3dv(pointD);
    glVertex3dv(pointC);
    glVertex3dv(pointG);
    glVertex3dv(pointH);
    glEnd();
    
    glColor3d(1.0, 0.5, 0.0);
    glBegin(GL_POLYGON);
    glVertex3dv(pointE);
    glVertex3dv(pointF);
    glVertex3dv(pointG);
    glVertex3dv(pointH);
    glEnd();
}



