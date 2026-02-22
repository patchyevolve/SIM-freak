#pragma once
// =============================================================================
//  render/GLHelper.h  — Minimal OpenGL loader for Compute Shaders
// =============================================================================

#include <windows.h>
#include <GL/gl.h>
#include <string>
#include <cstddef>

// Missing types in Windows gl.h
typedef char GLchar;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

// OpenGL Constants
#define GL_COMPUTE_SHADER                 0x91B9
#define GL_SHADER_STORAGE_BUFFER          0x90D2
#define GL_READ_WRITE                      0x88BA
#define GL_MAP_READ_BIT                   0x0001
#define GL_MAP_WRITE_BIT                  0x0002
#define GL_MAP_PERSISTENT_BIT             0x0040
#define GL_MAP_COHERENT_BIT               0x0080
#define GL_DYNAMIC_STORAGE_BIT            0x0100
#define GL_SHADER_STORAGE_BARRIER_BIT     0x00002000
#define GL_STREAM_DRAW                    0x88E0
#define GL_STATIC_DRAW                    0x88E4
#define GL_DYNAMIC_DRAW                   0x88E8
#define GL_COMPILE_STATUS                 0x8B81
#define GL_LINK_STATUS                    0x8B82
#define GL_INFO_LOG_LENGTH                0x8B84

// Function Types
typedef void   (WINAPI * PFNGLDISPATCHCOMPUTEPROC) (GLuint num_groups_x, GLuint num_groups_y, GLuint num_groups_z);
typedef void   (WINAPI * PFNGLMEMORYBARRIERPROC) (GLbitfield barriers);
typedef GLuint (WINAPI * PFNGLCREATESHADERPROC) (GLenum type);
typedef void   (WINAPI * PFNGLSHADERSOURCEPROC) (GLuint shader, GLsizei count, const GLchar* const* string, const GLint* length);
typedef void   (WINAPI * PFNGLCOMPILESHADERPROC) (GLuint shader);
typedef void   (WINAPI * PFNGLGETSHADERIVPROC) (GLuint shader, GLenum pname, GLint* params);
typedef void   (WINAPI * PFNGLGETSHADERINFOLOGPROC) (GLuint shader, GLsizei bufSize, GLsizei* length, GLchar* infoLog);
typedef GLuint (WINAPI * PFNGLCREATEPROGRAMPROC) (void);
typedef void   (WINAPI * PFNGLATTACHSHADERPROC) (GLuint program, GLuint shader);
typedef void   (WINAPI * PFNGLLINKPROGRAMPROC) (GLuint program);
typedef void   (WINAPI * PFNGLGETPROGRAMIVPROC) (GLuint program, GLenum pname, GLint* params);
typedef void   (WINAPI * PFNGLGETPROGRAMINFOLOGPROC) (GLuint program, GLsizei bufSize, GLsizei* length, GLchar* infoLog);
typedef void   (WINAPI * PFNGLUSEPROGRAMPROC) (GLuint program);
typedef void   (WINAPI * PFNGLDELETESHADERPROC) (GLuint shader);
typedef void   (WINAPI * PFNGLDELETEPROGRAMPROC) (GLuint program);
typedef void   (WINAPI * PFNGLGENBUFFERSPROC) (GLsizei n, GLuint* buffers);
typedef void   (WINAPI * PFNGLBINDBUFFERPROC) (GLenum target, GLuint buffer);
typedef void   (WINAPI * PFNGLBUFFERDATAPROC) (GLenum target, GLsizeiptr size, const void* data, GLenum usage);
typedef void   (WINAPI * PFNGLBUFFERSUBDATAPROC) (GLenum target, GLintptr offset, GLsizeiptr size, const void* data);
typedef void   (WINAPI * PFNGLBINDBUFFERBASEPROC) (GLenum target, GLuint index, GLuint buffer);
typedef void   (WINAPI * PFNGLDELETEBUFFERSPROC) (GLsizei n, const GLuint* buffers);
typedef void*  (WINAPI * PFNGLMAPBUFFERRANGEPROC) (GLenum target, GLintptr offset, GLsizeiptr length, GLbitfield access);
typedef GLboolean (WINAPI * PFNGLUNMAPBUFFERPROC) (GLenum target);
typedef void   (WINAPI * PFNGLUNIFORM1IPROC) (GLint location, GLint v0);
typedef void   (WINAPI * PFNGLUNIFORM1FPROC) (GLint location, GLfloat v0);
typedef GLint  (WINAPI * PFNGLGETUNIFORMLOCATIONPROC) (GLuint program, const GLchar* name);

// Externs
extern PFNGLDISPATCHCOMPUTEPROC glDispatchCompute;
extern PFNGLMEMORYBARRIERPROC   glMemoryBarrier;
extern PFNGLCREATESHADERPROC    glCreateShader;
extern PFNGLSHADERSOURCEPROC    glShaderSource;
extern PFNGLCOMPILESHADERPROC   glCompileShader;
extern PFNGLGETSHADERIVPROC     glGetShaderiv;
extern PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog;
extern PFNGLCREATEPROGRAMPROC   glCreateProgram;
extern PFNGLATTACHSHADERPROC    glAttachShader;
extern PFNGLLINKPROGRAMPROC     glLinkProgram;
extern PFNGLGETPROGRAMIVPROC    glGetProgramiv;
extern PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog;
extern PFNGLUSEPROGRAMPROC      glUseProgram;
extern PFNGLDELETESHADERPROC    glDeleteShader;
extern PFNGLDELETEPROGRAMPROC   glDeleteProgram;
extern PFNGLGENBUFFERSPROC      glGenBuffers;
extern PFNGLBINDBUFFERPROC      glBindBuffer;
extern PFNGLBUFFERDATAPROC      glBufferData;
extern PFNGLBUFFERSUBDATAPROC   glBufferSubData;
extern PFNGLBINDBUFFERBASEPROC  glBindBufferBase;
extern PFNGLDELETEBUFFERSPROC   glDeleteBuffers;
extern PFNGLMAPBUFFERRANGEPROC  glMapBufferRange;
extern PFNGLUNMAPBUFFERPROC     glUnmapBuffer;
extern PFNGLUNIFORM1IPROC       glUniform1i;
extern PFNGLUNIFORM1FPROC       glUniform1f;
extern PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation;

namespace GLHelper
{
    /// Load OpenGL functions. Must be called after a context is active.
    bool Init();

    /// Create a Compute Program from a shader file.
    GLuint CreateComputeProgram(const std::string& filename);
}
