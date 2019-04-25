#ifndef SHADER_HPP
#define SHADER_HPP

#include <GL/glew.h>

#include <GLFW/glfw3.h>

GLuint LoadShaders(const char * VertexSourceString,const char * FragmentSourceString, const char *goemetryShaderSource);

#endif
