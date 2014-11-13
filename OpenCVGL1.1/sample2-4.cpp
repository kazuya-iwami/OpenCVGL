#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <OpenGL/gl.h>
#include <GLUT/glut.h>

#define WINDOW_X (500)
#define WINDOW_Y (500)
#define WINDOW_NAME "test4"

void init_GL(int argc, char *argv[]);
void init();
void set_callback_functions();

void glut_display();
void glut_keyboard(unsigned char key, int x, int y);
void glut_mouse(int button, int state, int x, int y);
void glut_motion(int x, int y);

void draw_pyramid();
void draw_plane();

// グローバル変数
double g_angle1 = 0.0;
double g_angle2 = 0.0;
double g_angle3 = 0.0;
double g_angle4 = 3.141592/4;
double g_distance = 20.0;
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
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
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
    }else if(button == GLUT_RIGHT_BUTTON){
        if(state == GLUT_UP){
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
            g_angle2 += (double)(y - py)/20;
        }
        px = x;
        py = y;
    }else if(g_isRightButtonOn == true){
        if(px >= 0 && py >= 0){
            g_angle3 += (double)(x - px)/20;
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
    gluLookAt(g_distance * cos(g_angle2) * sin(g_angle1),
              g_distance * sin(g_angle2),
              g_distance * cos(g_angle2) * cos(g_angle1),
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    
    GLfloat lightpos[]={static_cast<GLfloat>(5 * cos(g_angle4) * sin(g_angle3)),
        static_cast<GLfloat>(5 * sin(g_angle4)),
        static_cast<GLfloat>(5 * cos(g_angle4) * cos(g_angle3)),
        1.0};
    GLfloat diffuse[] = {1.0, 1.0, 1.0, 1.0};
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    
    glPushMatrix();
    glTranslatef(5 * cos(g_angle4) * sin(g_angle3), 5 * sin(g_angle4),
                 5 * cos(g_angle4) * cos(g_angle3));
    glutSolidSphere(0.2, 50, 50);
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(5 * cos(g_angle4) * sin(g_angle4), 5 * sin(g_angle4),
                 5 * cos(g_angle4) * cos(g_angle4));
    glutSolidSphere(0.2, 50, 50);
    glPopMatrix();
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    glLightfv(GL_LIGHT0, GL_POSITION,lightpos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,diffuse);
    
    GLfloat lightpos2[]={static_cast<GLfloat>(10 * cos(g_angle4) * sin(g_angle4)),
        static_cast<GLfloat>(10 * sin(g_angle4)),
        static_cast<GLfloat>(10 * cos(g_angle4) * cos(g_angle4)),
        1.0};
    
   


    
    glEnable(GL_LIGHT1);
    
    glLightfv(GL_LIGHT1, GL_POSITION,lightpos2);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,diffuse);
    
    
    glPushMatrix();
    glTranslatef(0.0, -2.0, 0.0);
    draw_plane();
    glPopMatrix();
    
    glPushMatrix();
    glScalef(1.0, 2.0, 1.0);
    draw_pyramid();
    glPopMatrix();
    
    glFlush();
    
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    glutSwapBuffers();
}


void draw_pyramid(){
    GLdouble pointO[] = {0.0, 1.0, 0.0};
    GLdouble pointA[] = {1.5, -1.0, 1.5};
    GLdouble pointB[] = {-1.5, -1.0, 1.5};
    GLdouble pointC[] = {-1.5, -1.0, -1.5};
    GLdouble pointD[] = {1.5, -1.0, -1.5};
    
    GLfloat facecolor1[] = {1.0, 0.0, 0.0, 0.8};
    GLfloat facecolor2[] = {1.0, 1.0, 0.0, 0.8};
    GLfloat facecolor3[] = {0.0, 1.0, 1.0, 0.8};
    GLfloat facecolor4[] = {1.0, 0.0, 1.0, 0.8};
    GLfloat facecolor5[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat specular[] = {0.0, 0.0, 0.0, 1.0};
    
    glMaterialfv(GL_FRONT, GL_DIFFUSE, facecolor1);
    glNormal3d(0.0, 0.6, 0.8);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointA);
    glVertex3dv(pointB);
    glEnd();
    
    glMaterialfv(GL_FRONT, GL_DIFFUSE, facecolor2);
    glNormal3d(-0.8, 0.6, 0.0);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointB);
    glVertex3dv(pointC);
    glEnd();
    
    glMaterialfv(GL_FRONT, GL_DIFFUSE, facecolor3);
    glNormal3d(0.0, 0.6, -0.8);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointC);
    glVertex3dv(pointD);
    glEnd();
    
    glMaterialfv(GL_FRONT, GL_DIFFUSE, facecolor4);
    glNormal3d(0.8, 0.6, 0.0);
    glBegin(GL_TRIANGLES);
    glVertex3dv(pointO);
    glVertex3dv(pointD);
    glVertex3dv(pointA);
    glEnd();
    
    glMaterialfv(GL_FRONT, GL_DIFFUSE, facecolor5);
    glNormal3d(0.0, -1.0, 0.0);
    glBegin(GL_POLYGON);
    glVertex3dv(pointA);
    glVertex3dv(pointB);
    glVertex3dv(pointC);
    glVertex3dv(pointD);
    glEnd();
}

void draw_plane(){
    GLfloat facecolor[] = {0.9, 0.9, 0.9, 1.0};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, facecolor);
    glNormal3d(0.0, 1.0, 0.0);
    glBegin(GL_QUADS);
    glVertex3d(4.0, 0.0, 4.0);
    glVertex3d(4.0, 0.0, -4.0);
    glVertex3d(-4.0, 0.0, -4.0);
    glVertex3d(-4.0, 0.0, 4.0);
    glEnd();
}