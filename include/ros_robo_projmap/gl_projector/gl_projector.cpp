#include <Python.h>
#include <numpy/arrayobject.h>
#include <stdlib.h>

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "common/shader.hpp"

#include <time.h>
using namespace glm;

/**
 * Much of this file is adapted from the following tutorials:
 * http://www.opengl-tutorial.org/intermediate-tutorials/billboards-particles/particles-instancing/
 * https://www.opengl-tutorial.org/intermediate-tutorials/tutorial-9-vbo-indexing/
 */

#define MAX_INPUT_WIDTH     1920
#define MAX_INPUT_HEIGHT    1080
#define MAX_INPUT_RES       MAX_INPUT_WIDTH*MAX_INPUT_HEIGHT
#define MAX_XY_SIZE         2*MAX_INPUT_RES
#define MAX_D_SIZE          MAX_INPUT_RES
#define MAX_RGB_SIZE        3*MAX_INPUT_RES
#define MAX_INDICES_SIZE    3*2*(MAX_INPUT_WIDTH - 1)*(MAX_INPUT_HEIGHT - 1)

GLFWwindow* window;
GLuint programID;
GLint MatrixID;
glm::mat4 MVP;
GLuint attrArrayIDs[3];

PyObject *g_arr_xy;
GLuint vertexbuffer;
GLuint depthbuffer;
GLuint rgbbuffer;
GLuint indexbuffer;

static unsigned int g_indices_data[MAX_INDICES_SIZE];

void generate_indices(int width, int height, unsigned int indices[]);

/** Shader source codes **/
char const *vertexShaderSource = 
    "#version 330 core\n"
    // Input vertex data, different for all executions of this shader.
    "layout(location = 0) in vec2 vertexPosition_modelspace;\n"
    // Depth corresponding to vertex data
    "layout(location = 1) in float depth;\n"
    // Input color data (per vertex), passed straight through to the fragment shader
    "layout(location = 2) in vec3 colorIn;\n"
    "out vec3 colorOut;\n"
    // Values that stay constant for the whole mesh.
    "uniform mat4 MVP;\n"
    "void main() {\n"
        "colorOut = colorIn;\n"
        // Output position of the vertex, in clip space : MVP * position
        "gl_Position =  MVP * vec4(vertexPosition_modelspace * depth, depth, 1);\n"
    "}";

char const *fragmentShaderSource = 
    "#version 330 core\n"
    "in vec3 colorOut;\n" // color from vertex shader
    "out vec3 color;\n" // output color
    "void main() {\n"
	    "color = colorOut;\n"
    "}";

/**
 * Takes in (np.ndarray projector matrix, np.ndarray pixel xy indices, 
 *  input_width, input_height, proj_width, proj_height, output monitor index).
 * The projector matrix must be 4x4, and output monitor can be negative for windowed mode.
 * Returns True on success, False otherwise
 */
PyObject *start(PyObject *self, PyObject *args) {

    // Extract Python args

    // read args
    PyObject *arg_mvp = NULL;
    PyObject *arg_xy = NULL;
    PyObject *arr_mvp = NULL;
    int input_width;
    int input_height;
    int proj_width;
    int proj_height;
    int monitor_index;

    if (!PyArg_ParseTuple(args, "OOiiiii", &arg_mvp, &arg_xy,
        &input_width, &input_height, &proj_width, &proj_height, &monitor_index)) {
        Py_RETURN_NONE;
    }

    arr_mvp = PyArray_FROM_OTF(arg_mvp, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);
    g_arr_xy = PyArray_FROM_OTF(arg_xy, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);

    size_t mvp_size = PyArray_Size(arr_mvp);
    size_t xy_size = PyArray_Size(g_arr_xy);
    if (mvp_size != 16) {
            fprintf(stderr, "Invalid size for MVP matrix\n");
            Py_RETURN_NONE;
    }
    if (xy_size != (input_width * input_height * 2)) {
            fprintf(stderr, "Invalid size for xy size\n");
            Py_RETURN_NONE;
    }

    // Copy input python data into our matrix object
    float *mvp_data = (float*) PyArray_DATA(arr_mvp);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            MVP[j][i] = *(mvp_data + i*4 + j);
        }
    }
    // decrement reference on arguments
    Py_DECREF(arr_mvp);

    // Initialise GLFW
    if( !glfwInit() ) {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        Py_RETURN_FALSE;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_RESIZABLE,GL_FALSE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(proj_width, proj_height, "Projector View", NULL, NULL);
    if( window == NULL ) {
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        glfwTerminate();
        Py_RETURN_FALSE;
    }
    glfwMakeContextCurrent(window);

    // Set the window to be fullscreened in the given monitor
    if (monitor_index >= 0) {
        int count;
        GLFWmonitor** monitors = glfwGetMonitors(&count);
        if (monitor_index >= count) {
            fprintf(stderr, "Not enough monitors (%d) to project to monitor at index %d\n", count, monitor_index);
            Py_RETURN_FALSE;
        }
        glfwSetWindowMonitor(window, monitors[monitor_index], 0, 0, proj_width, proj_height, GLFW_DONT_CARE);
    }

    // Initialize GLEW
    glewExperimental = true;
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        glfwTerminate();
		Py_RETURN_FALSE;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Light background
    glClearColor(0.8f, 0.8f, 0.9f, 0.0f);

    // Setup vertex arrays and associated buffers
    glGenVertexArrays(3, attrArrayIDs);
    glBindVertexArray(attrArrayIDs[0]);
    glGenBuffers(3, attrArrayIDs);
    vertexbuffer =  attrArrayIDs[0];
    depthbuffer =   attrArrayIDs[1];
    rgbbuffer =     attrArrayIDs[2];

    // Create and compile our GLSL program from the shaders
    programID = LoadShaders(vertexShaderSource, fragmentShaderSource);

    // Get a handle for our "MVP" uniform
    MatrixID = glGetUniformLocation(programID, "MVP");

    // create AND populate buffer for the vertices (XY)
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    GLfloat *g_xy_data =  (GLfloat*) PyArray_DATA(g_arr_xy);
    glBufferData(GL_ARRAY_BUFFER, xy_size * sizeof(GLfloat), g_xy_data, GL_STATIC_DRAW);
    // don't decref g_xy_data until close
    
    // create buffer for depth vals
    glBindBuffer(GL_ARRAY_BUFFER, depthbuffer);
    glBufferData(GL_ARRAY_BUFFER, MAX_D_SIZE * sizeof(GLfloat), NULL, GL_DYNAMIC_DRAW);

    // create buffer for colors (RGB)
    glBindBuffer(GL_ARRAY_BUFFER, rgbbuffer);
    glBufferData(GL_ARRAY_BUFFER, MAX_RGB_SIZE * sizeof(GLfloat), NULL, GL_DYNAMIC_DRAW);

    // Populate the indices array
    generate_indices(input_width, input_height, g_indices_data);

    // Generate a buffer for the indices of the triangle points
    glGenBuffers(1, &indexbuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexbuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_INDICES_SIZE * sizeof(int), g_indices_data, GL_STATIC_DRAW);
    
    Py_RETURN_TRUE;
}

void generate_indices(int width, int height, unsigned int indices[]) {
    int i = 0;

    for (int y = 0; y < height - 1; y++) {
        for (int x = 0; x < width - 1; x++) {
            int ind = y * width + x;
            int ind_r = ind + 1;
            int ind_d = ind + width;
            int ind_dr = ind + 1 + width;

            indices[i + 0] = ind;
            indices[i + 1] = ind_r;
            indices[i + 2] = ind_d;
            indices[i + 3] = ind_dr;
            indices[i + 4] = ind_d;
            indices[i + 5] = ind_r;

            i += 6;
        }
    }
}

int check_for_exit();

/**
 * Takes in (np.ndarray depth, np.ndarray rgb).
 * with N words of (x, y), (d), (r, g, b), respectively
 * where N is the number of pixels, x, y are the pixel locations.
 * Returns true when the window was closed or None on error.
 */
PyObject *draw_frame(PyObject *self, PyObject *args) {
    // read args
    PyObject *arg_d = NULL; PyObject *arg_rgb = NULL;
    PyObject *arr_d = NULL; PyObject *arr_rgb = NULL;

    if (!PyArg_ParseTuple(args, "OO", &arg_d, &arg_rgb)) {
        Py_RETURN_NONE;
    }

    arr_d =     PyArray_FROM_OTF(arg_d, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);
    arr_rgb =   PyArray_FROM_OTF(arg_rgb, NPY_FLOAT32, NPY_ARRAY_IN_ARRAY);

    int d_size =    PyArray_Size(arr_d);
    int rgb_size =  PyArray_Size(arr_rgb);
    if (d_size > MAX_D_SIZE || rgb_size > MAX_RGB_SIZE) {
            fprintf(stderr, "Invalid input sizes to draw_frame\n");
            Py_RETURN_NONE;
    }

    GLfloat *d_data =   (GLfloat*) PyArray_DATA(arr_d);
    GLfloat *rgb_data = (GLfloat*) PyArray_DATA(arr_rgb);

    // reset color
    glClear(GL_COLOR_BUFFER_BIT);

    // Use our shader (the camera->projector conversion matrix and the color attribute adder)
    glUseProgram(programID);
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

    // simply enable the already-populated XY pixel location array
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
            0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
            2,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            2*sizeof(GLfloat),  // stride
            (void*)0            // array buffer offset
    );

    /* populate d buffer and assign to attribute */
    // switch current GL_ARRAY_BUFFER binding to the correct array
    glBindBuffer(GL_ARRAY_BUFFER, depthbuffer);
    // update data in buffers (reallocate the buffer objects beforehand for speed)
    // (see https://www.khronos.org/opengl/wiki/Buffer_Object_Streaming#Buffer_re-specification)
    glBufferData(GL_ARRAY_BUFFER, MAX_D_SIZE * sizeof(GLfloat), NULL, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, d_size * sizeof(GLfloat), d_data);
    glEnableVertexAttribArray(1);
    // connect to shader attribute index
    glVertexAttribPointer(
            1,
            1,                                      // size (single)
            GL_FLOAT,
            GL_FALSE,                               // normalized?
            sizeof(GLfloat),                        // stride
            (void*)0
    );

    /* populate rgb buffer and assign to attribute */
    glBindBuffer(GL_ARRAY_BUFFER, rgbbuffer);
    glBufferData(GL_ARRAY_BUFFER, MAX_RGB_SIZE * sizeof(GLfloat), NULL, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, rgb_size * sizeof(GLfloat), rgb_data);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
            2,
            3,                                      // size (RGB)
            GL_FLOAT,
            GL_FALSE,                               // normalized?
            3*sizeof(GLfloat),                      // stride
            (void*)0
    );

    // Draw the triangles using the vertex indices in the indices data
    glBindVertexArray(attrArrayIDs[0]);
    glDrawElements(
        GL_TRIANGLES,               // mode
        MAX_INDICES_SIZE,           // count
        GL_UNSIGNED_INT,            // type
        (void*)0                    // element array buffer offset
    );

    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    // Swap buffers
    glfwSwapBuffers(window);

    // decrement reference on arguments
    Py_DECREF(arr_d); 
    Py_DECREF(arr_rgb); 

    if (check_for_exit()) {
        Py_RETURN_TRUE;
    } else {
        Py_RETURN_FALSE;
    }
}

int check_for_exit() {
    // Check for exit keypresses
    glfwPollEvents();
	if (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == 0) {
		return 0;
	} else {
        	return 1;
	}
}

/**
 * Stops rendering
 */
PyObject *stop(PyObject *self, PyObject *args) {
    Py_DECREF(g_arr_xy);
    glfwTerminate();
	Py_RETURN_NONE;
}

static PyMethodDef Gl_ProjectorMethods[] = {
	{"start",  start, METH_VARARGS, "Starts the projector display at the given resolution, using the given camera->projector conversion matrix"},
	{"draw_frame",  draw_frame, METH_VARARGS, "Transforms the given frame and corresponding triangle indices into a mesh which is then projected and displayed"},
	{"stop",  stop, METH_NOARGS, "Stops the projector display"},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

extern "C" {
    void initgl_projector(void) {
        import_array();
        Py_InitModule("gl_projector", Gl_ProjectorMethods);
    }
}
