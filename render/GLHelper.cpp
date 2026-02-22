// =============================================================================
//  render/GLHelper.cpp
// =============================================================================

#include "GLHelper.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

PFNGLDISPATCHCOMPUTEPROC glDispatchCompute = nullptr;
PFNGLMEMORYBARRIERPROC   glMemoryBarrier   = nullptr;
PFNGLCREATESHADERPROC    glCreateShader    = nullptr;
PFNGLSHADERSOURCEPROC    glShaderSource    = nullptr;
PFNGLCOMPILESHADERPROC   glCompileShader   = nullptr;
PFNGLGETSHADERIVPROC     glGetShaderiv     = nullptr;
PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog = nullptr;
PFNGLCREATEPROGRAMPROC   glCreateProgram   = nullptr;
PFNGLATTACHSHADERPROC    glAttachShader    = nullptr;
PFNGLLINKPROGRAMPROC     glLinkProgram     = nullptr;
PFNGLGETPROGRAMIVPROC    glGetProgramiv    = nullptr;
PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog = nullptr;
PFNGLUSEPROGRAMPROC      glUseProgram      = nullptr;
PFNGLDELETESHADERPROC    glDeleteShader    = nullptr;
PFNGLDELETEPROGRAMPROC   glDeleteProgram   = nullptr;
PFNGLGENBUFFERSPROC      glGenBuffers      = nullptr;
PFNGLBINDBUFFERPROC      glBindBuffer      = nullptr;
PFNGLBUFFERDATAPROC      glBufferData      = nullptr;
PFNGLBINDBUFFERBASEPROC  glBindBufferBase  = nullptr;
PFNGLDELETEBUFFERSPROC   glDeleteBuffers   = nullptr;
PFNGLMAPBUFFERRANGEPROC  glMapBufferRange  = nullptr;
PFNGLUNMAPBUFFERPROC     glUnmapBuffer     = nullptr;
PFNGLUNIFORM1IPROC       glUniform1i       = nullptr;
PFNGLUNIFORM1FPROC       glUniform1f       = nullptr;
PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation = nullptr;

namespace GLHelper
{
    bool Init()
    {
        glDispatchCompute = (PFNGLDISPATCHCOMPUTEPROC)wglGetProcAddress("glDispatchCompute");
        glMemoryBarrier   = (PFNGLMEMORYBARRIERPROC)wglGetProcAddress("glMemoryBarrier");
        glCreateShader    = (PFNGLCREATESHADERPROC)wglGetProcAddress("glCreateShader");
        glShaderSource    = (PFNGLSHADERSOURCEPROC)wglGetProcAddress("glShaderSource");
        glCompileShader   = (PFNGLCOMPILESHADERPROC)wglGetProcAddress("glCompileShader");
        glGetShaderiv     = (PFNGLGETSHADERIVPROC)wglGetProcAddress("glGetShaderiv");
        glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC)wglGetProcAddress("glGetShaderInfoLog");
        glCreateProgram   = (PFNGLCREATEPROGRAMPROC)wglGetProcAddress("glCreateProgram");
        glAttachShader    = (PFNGLATTACHSHADERPROC)wglGetProcAddress("glAttachShader");
        glLinkProgram     = (PFNGLLINKPROGRAMPROC)wglGetProcAddress("glLinkProgram");
        glGetProgramiv    = (PFNGLGETPROGRAMIVPROC)wglGetProcAddress("glGetProgramiv");
        glGetProgramInfoLog = (PFNGLGETPROGRAMINFOLOGPROC)wglGetProcAddress("glGetProgramInfoLog");
        glUseProgram      = (PFNGLUSEPROGRAMPROC)wglGetProcAddress("glUseProgram");
        glDeleteShader    = (PFNGLDELETESHADERPROC)wglGetProcAddress("glDeleteShader");
        glDeleteProgram   = (PFNGLDELETEPROGRAMPROC)wglGetProcAddress("glDeleteProgram");
        glGenBuffers      = (PFNGLGENBUFFERSPROC)wglGetProcAddress("glGenBuffers");
        glBindBuffer      = (PFNGLBINDBUFFERPROC)wglGetProcAddress("glBindBuffer");
        glBufferData      = (PFNGLBUFFERDATAPROC)wglGetProcAddress("glBufferData");
        glBindBufferBase  = (PFNGLBINDBUFFERBASEPROC)wglGetProcAddress("glBindBufferBase");
        glDeleteBuffers   = (PFNGLDELETEBUFFERSPROC)wglGetProcAddress("glDeleteBuffers");
        glMapBufferRange  = (PFNGLMAPBUFFERRANGEPROC)wglGetProcAddress("glMapBufferRange");
        glUnmapBuffer     = (PFNGLUNMAPBUFFERPROC)wglGetProcAddress("glUnmapBuffer");
        glUniform1i       = (PFNGLUNIFORM1IPROC)wglGetProcAddress("glUniform1i");
        glUniform1f       = (PFNGLUNIFORM1FPROC)wglGetProcAddress("glUniform1f");
        glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC)wglGetProcAddress("glGetUniformLocation");

        if (!glDispatchCompute || !glMemoryBarrier || !glCreateShader || !glGenBuffers || !glMapBufferRange || !glUniform1f)
        {
            std::cerr << "[Error] GLHelper::Init failed. Required functions not found.\n";
            return false;
        }
        return true;
    }

    GLuint CreateComputeProgram(const std::string& filename)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[Error] Failed to open shader file: " << filename << "\n";
            return 0;
        }
        std::stringstream ss;
        ss << file.rdbuf();
        std::string sourceStr = ss.str();
        const char* sourceCode = sourceStr.c_str();

        GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &sourceCode, nullptr);
        glCompileShader(shader);

        GLint success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(shader, 512, nullptr, infoLog);
            std::cerr << "[Error] Compute Shader compilation failed (" << filename << "):\n" << infoLog << "\n";
            return 0;
        }

        GLuint program = glCreateProgram();
        glAttachShader(program, shader);
        glLinkProgram(program);

        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(program, 512, nullptr, infoLog);
            std::cerr << "[Error] Compute Program linking failed (" << filename << "):\n" << infoLog << "\n";
            return 0;
        }

        glDeleteShader(shader); // flagged for deletion
        return program;
    }
}
