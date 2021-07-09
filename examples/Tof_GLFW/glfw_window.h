// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <map>
using namespace PointCloud;
#define PI 3.14159265358979323846
#define IMU_FRAME_WIDTH 1280
#define IMU_FRAME_HEIGHT 720
//////////////////////////////
// Basic Data Types         //
//////////////////////////////
enum render_format
{
    FORMAT_UNKOWN = 0,
    FORMAT_RGB8 = 1,
    FORMAT_RGBA8 = 2,
    FORMAT_Y8 = 3
};
struct vertex {
    float x, y, z;
    operator const float*() const { return &x; }
};
struct texture_coordinate {
    float u, v;
    operator const float*() const { return &u; }
};

struct float3 { 
    float x, y, z; 
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};
struct float2 { float x, y; };

struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const
    {
        auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
        if (W > w)
        {
            auto scale = w / W;
            W *= scale;
            H *= scale;
        }

        return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
    }
};



class rs2_frame
{
    
};

#ifdef FRAME_TYPE
class Frame
{
public:
    /**
     * Base class for multiple Frame extensions
     */
    Frame() : frame_ref(nullptr) {}
    /**
     * Base class for multiple frame extensions with internal frame handle
     * \param[in] rs2_frame frame_ref - internal frame instance
     */
    Frame(rs2_frame* ref) : frame_ref(ref)
    {
    }
    /**
     * Change the internal frame handle to the one in parameter, then put the other frame internal frame handle to nullptr
     * \param[in] frame other - another frame instance to be pointed to
     */
    Frame(Frame&& other) noexcept : frame_ref(other.frame_ref)
    {
        other.frame_ref = nullptr;
    }
    /**
     * Change the internal frame handle to the one in parameter, the function exchange the internal frame handle.
     * \param[in] frame other - another frame instance to be pointed to
     */
    Frame& operator=(Frame other)
    {
        swap(other);
        return *this;
    }
    
    /**
     * Set the internal frame handle to the one in parameter, the function create additional reference if internal reference exist.
     * \param[in] frame other - another frame instance to be pointed to
     */
    Frame(const Frame& other)
    : frame_ref(other.frame_ref)
    {
        //if (frame_ref) add_ref();

    }
    /**
     * Swap the internal frame handle with the one in parameter
     * \param[in] frame other - another frame instance to be swaped
     */
    void swap(Frame& other)
    {
        std::swap(frame_ref, other.frame_ref);
        
    }
    
    /**
     * releases the frame handle
     */
    ~Frame()
    {
        if (frame_ref)
        {
            //rs2_release_frame(frame_ref);
        }
    }
    
    /**
     * keep the frame, otherwise if no refernce to the frame, the frame will be released.
     */
    void keep() {
        //rs2_keep_frame(frame_ref);
        
    }
    
    /**
     * Parenthesis operator check internal frame handle is valid.
     * \return bool - true or false.
     */
    operator bool() const { return frame_ref != nullptr; }
    
    /**
     * retrieve the time at which the frame was captured
     * \return            the timestamp of the frame, in milliseconds since the device was started
     */
    double get_timestamp() const
    {
        /*
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_timestamp(frame_ref, &e);
        error::handle(e);
        return r;*/
        return 0;
    }
    
    /** retrieve the timestamp domain
     * \return            timestamp domain (clock name) for timestamp values
     */
    /*
    rs2_timestamp_domain get_frame_timestamp_domain() const
    {
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_timestamp_domain(frame_ref, &e);
        error::handle(e);
        return r;
    }*/
    
    /** retrieve the current value of a single frame_metadata
     * \param[in] frame_metadata  the frame_metadata whose value should be retrieved
     * \return            the value of the frame_metadata
     */
    /*
    rs2_metadata_type get_frame_metadata(rs2_frame_metadata_value frame_metadata) const
    {
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_metadata(frame_ref, frame_metadata, &e);
        error::handle(e);
        return r;
    }*/
    
    /** determine if the device allows a specific metadata to be queried
     * \param[in] frame_metadata  the frame_metadata to check for support
     * \return            true if the frame_metadata can be queried
     */
    /*
    bool supports_frame_metadata(rs2_frame_metadata_value frame_metadata) const
    {
        rs2_error* e = nullptr;
        auto r = rs2_supports_frame_metadata(frame_ref, frame_metadata, &e);
        error::handle(e);
        return r != 0;
    }*/
    
    /**
     * retrieve frame number (from frame handle)
     * \return               the frame number of the frame, in milliseconds since the device was started
     */
    /*
    unsigned long long get_frame_number() const
    {
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_number(frame_ref, &e);
        error::handle(e);
        return r;
    }
    */
    /**
     * retrieve data from frame handle
     * \return               the pointer to the start of the frame data
     */
    
    const void* get_data() const
    {
        /*
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_data(frame_ref, &e);
        error::handle(e);
        return r;*/
        return 0;
    }
    
    /**
     * retrieve stream profile from frame handle
     * \return  stream_profile - the pointer to the stream profile
     */
    /*
    stream_profile get_profile() const
    {
        rs2_error* e = nullptr;
        auto s = rs2_get_frame_stream_profile(frame_ref, &e);
        error::handle(e);
        return stream_profile(s);
    }*/
    
    /**
     * Template function, checking if current instance is the type of another class
     * \return  bool - true or false.
     */
    template<class T>
    bool is() const
    {
        T extension(*this);
        return extension;
    }
    /**
     * Template function, cast current instance as the type of another class
     * \return  class instance.
     */
    template<class T>
    T as() const
    {
        T extension(*this);
        return extension;
    }
    
    /**
     * Retrieve back the internal frame handle
     * \return  rs2_frame - internal frame handle.
     */
    rs2_frame* get() const { return frame_ref; }
    explicit operator rs2_frame*() { return frame_ref; }
    /*
    frame apply_filter(filter_interface& filter)
    {
        return filter.process(*this);
    }*/
    
protected:
    /**
     * add additional reference to a frame without duplicating frame data
     * \param[out] result     new frame reference, release by destructor
     * \return                true if cloning was successful
     */
    /*
    void add_ref() const
    {
        rs2_error* e = nullptr;
        rs2_frame_add_ref(frame_ref, &e);
        error::handle(e);
    }
    
    void reset()
    {
        if (frame_ref)
        {
            rs2_release_frame(frame_ref);
        }
        frame_ref = nullptr;
    }*/
    
private:
    rs2_frame* frame_ref;
};



class Points : public Frame
{
public:
    /**
     * Inherit frame class with additional point cloud related attributs/functions
     */
    Points() : Frame(), _size(0) {}
    
    /**
     * Inherit frame class with additional point cloud related attributs/functions
     * \param[in] frame - existing frame instance
     */
    Points(const Frame& f)
    : Frame(f), _size(0)
    {
        /*
        rs2_error* e = nullptr;
        if (!f || (rs2_is_frame_extendable_to(f.get(), RS2_EXTENSION_POINTS, &e) == 0 && !e))
        {
            reset();
        }
        error::handle(e);
        
        if (get())
        {
            _size = rs2_get_frame_points_count(get(), &e);
            error::handle(e);
        }*/
    }
    /**
     * Retrieve back the vertices
     * \param[in] vertex* - pointer of vertex sturcture
     */

    const vertex* get_vertices() const
    {
        /*
        rs2_error* e = nullptr;
        auto res = rs2_get_frame_vertices(get(), &e);
        error::handle(e);
        return (const vertex*)res;
         */
        return 0;
    }

    /**
     * Export current point cloud to PLY file
     * \param[in] string fname - file name of the PLY to be saved
     * \param[in] video_frame texture - the texture for the PLY.
     */
    /*
    void export_to_ply(const std::string& fname, video_frame texture)
    {
        rs2_frame* ptr = nullptr;
        std::swap(texture.frame_ref, ptr);
        rs2_error* e = nullptr;
        rs2_export_to_ply(get(), fname.c_str(), ptr, &e);
        error::handle(e);
    }*/
    /**
     * return the texture coordinate(uv map) for the point cloud
     * \return texture_coordinate* - pointer of texture coordinates.
     */
    
    const texture_coordinate* get_texture_coordinates() const
    {
        /*
        rs2_error* e = nullptr;
        auto res = rs2_get_frame_texture_coordinates(get(), &e);
        error::handle(e);
        return (const texture_coordinate*)res;
        */
        return 0;
    }
    
    size_t size() const
    {
        return _size;
    }
    
private:
    size_t _size;
};


class video_frame : public Frame
{
public:
    /**
     * Inherit frame class with additional video related attributs/functions
     * \param[in] frame - existing frame instance
     */
    video_frame(const Frame& f)
    : Frame(f)
    {
        /*
        rs2_error* e = nullptr;
        if (!f || (rs2_is_frame_extendable_to(f.get(), RS2_EXTENSION_VIDEO_FRAME, &e) == 0 && !e))
        {
            reset();
        }
        error::handle(e);*/
    }
    
    
    /**
     * returns image width in pixels
     * \return        frame width in pixels
     */
    int get_width() const
    {
        /*
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_width(get(), &e);
        error::handle(e);
        return r;*/
        return 0;
    }
    
    /**
     * returns image height in pixels
     * \return        frame height in pixels
     */
    int get_height() const
    {
        /*
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_height(get(), &e);
        error::handle(e);
        return r;*/
        return 0;
    }
    
    /**
     * retrieve frame stride, meaning the actual line width in memory in bytes (not the logical image width)
     * \return            stride in bytes
     */
    int get_stride_in_bytes() const
    {
        /*
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_stride_in_bytes(get(), &e);
        error::handle(e);*/
        return 0;
    }
    
    /**
     * retrieve bits per pixel
     * \return            number of bits per one pixel
     */
    int get_bits_per_pixel() const
    {/*
        rs2_error* e = nullptr;
        auto r = rs2_get_frame_bits_per_pixel(get(), &e);
        error::handle(e);
        */
        return 0;
    }
    
    /**
     * retrieve bytes per pixel
     * \return            number of bytes per one pixel
     */
    int get_bytes_per_pixel() const { return get_bits_per_pixel() / 8; }
};
#endif
//////////////////////////////
// Simple font loading code //
//////////////////////////////

#include "stb_easy_font.h"

inline void draw_text(int x, int y, const char * text)
{
    char buffer[60000]; // ~300 chars
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, buffer);
    glDrawArrays(GL_QUADS, 0, 4 * stb_easy_font_print((float)x, (float)(y - 7), (char *)text, nullptr, buffer, sizeof(buffer)));
    glDisableClientState(GL_VERTEX_ARRAY);
}

void set_viewport(const rect& r)
{
    glViewport( (int)r.x, (int)r.y, (int)r.w, (int)r.h);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, r.w, r.h, 0, -1, +1);
}

////////////////////////
// Image display code //
////////////////////////
class texture
{
    GLuint gl_handle = 0;
   // rs2_stream stream = RS2_STREAM_ANY;

public:
    
    void render(const RawDataFrame* frame, int width,int height,const rect& rect)
    {
        upload(frame,width,height);
        show(rect.adjust_ratio({ (float)width, (float)height }));
       // show(rect.adjust_ratio({ (float)frame.get_width(), (float)frame.get_height() }));
    }
    
    void upload(const RawDataFrame*  frame,  int width,int height)
    {
        if (!frame) return;

        if (!gl_handle)
            glGenTextures(1, &gl_handle);
        //GLenum err = glGetError();

        int format = frame->id;

        glBindTexture(GL_TEXTURE_2D, gl_handle);
        
        //glDrawPixels(640, 480, GL_RED, GL_SHORT,  frame->data.data());
        
        switch (format)
        {
        case FORMAT_RGB8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE,  frame->data.data());
            break;
        case FORMAT_RGBA8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,  frame->data.data());
            break;
        case FORMAT_Y8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE,  frame->data.data());
            break;
        default:
            throw std::runtime_error("The requested format is not supported by this demo!");
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void show(const rect& r) const
    {
        if (!gl_handle)
            return;

        set_viewport(r);

        glBindTexture(GL_TEXTURE_2D, gl_handle);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2f(0, 0);
        glTexCoord2f(0, 1); glVertex2f(0, r.h);
        glTexCoord2f(1, 1); glVertex2f(r.w, r.h);
        glTexCoord2f(1, 0); glVertex2f(r.w, 0);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
        //draw_text(int(0.05f * r.w), int(r.h - 0.05f*r.h), rs2_stream_to_string(stream));
    }

    GLuint get_gl_handle() { return gl_handle; }
};

class window
{
public:
    std::function<void(bool)>           on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)>            on_key_release = [](int) {};

    window(int width, int height, const char* title)
        : _width(width), _height(height)
    {
        glfwInit();
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
        glfwMakeContextCurrent(win);

        glfwSetWindowUserPointer(win, this);
        glfwSetMouseButtonCallback(win, [](GLFWwindow * w, int button, int action, int mods)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
        });

        glfwSetScrollCallback(win, [](GLFWwindow * w, double xoffset, double yoffset)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            s->on_mouse_scroll(xoffset, yoffset);
        });

        glfwSetCursorPosCallback(win, [](GLFWwindow * w, double x, double y)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            s->on_mouse_move(x, y);
        });

        glfwSetKeyCallback(win, [](GLFWwindow * w, int key, int scancode, int action, int mods)
        {
            auto s = (window*)glfwGetWindowUserPointer(w);
            if (0 == action) // on key release
            {
                s->on_key_release(key);
            }
        });
    }

    float width() const { return float(_width); }
    float height() const { return float(_height); }

    operator bool()
    {
        glPopMatrix();
        glfwSwapBuffers(win);

        auto res = !glfwWindowShouldClose(win);

        glfwPollEvents();
        glfwGetFramebufferSize(win, &_width, &_height);

        // Clear the framebuffer
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, _width, _height);

        // Draw the images
        glPushMatrix();
        glfwGetWindowSize(win, &_width, &_height);
        glOrtho(0, _width, _height, 0, -1, +1);

        return res;
    }

    ~window()
    {
        glfwDestroyWindow(win);
        glfwTerminate();
    }

    /*
    void show(Frame frame)
    {
        show(frame, { 0, 0, (float)_width, (float)_height });
    }*/

    void show(const Frame& frame, const rect& rect)
    {
        /*
        if (auto fs = frame.as<rs2::frameset>())
            render_frameset(fs, rect);
        if (auto vf = frame.as<rs2::video_frame>())
            render_video_frame(vf, rect);
        */
       // render_video_frame(frame, rect);

    }

    operator GLFWwindow*() { return win; }

private:
    GLFWwindow * win;
    std::map<int, texture> _textures;
    //std::map<int, imu_drawer> _imus;
    int _width, _height;
    /*
    void render_video_frame(const video_frame& f, const rect& r)
    {
        //auto& t = _textures[f.get_profile().unique_id()];
        auto& t = _textures[0];
        t.render(f, r);
    }

    
    void render_frameset(const rs2::frameset& frames, const rect& r)
    {
        std::vector<Frame> supported_frames;
        for (auto f : frames)
        {
            if (can_render(f))
                supported_frames.push_back(f);
        }
        if (supported_frames.empty())
            return;

        std::sort(supported_frames.begin(), supported_frames.end(), [](Frame first, Frame second)
        { return first.get_profile().stream_type() < second.get_profile().stream_type();  });

        auto image_grid = calc_grid(r, supported_frames);

        int image_index = 0;
        for (auto f : supported_frames)
        {
            auto r = image_grid.at(image_index);
            show(f, r);
            image_index++;
        }
    }*/

    bool can_render(const Frame& f) const
    {
        return true;
        /*
        auto format = f.get_profile().format();
        switch (format)
        {
        case RS2_FORMAT_RGB8:
        case RS2_FORMAT_RGBA8:
        case RS2_FORMAT_Y8:
        case RS2_FORMAT_MOTION_XYZ32F:
            return true;
        default:
            return false;
        }*/
    }

    rect calc_grid(rect r, size_t streams)
    {
        if (r.w <= 0 || r.h <= 0 || streams <= 0)
            throw std::runtime_error("invalid window configuration request, failed to calculate window grid");
        float ratio = r.w / r.h;
        auto x = sqrt(ratio * (float)streams);
        auto y = (float)streams / x;
        auto w = round(x);
        auto h = round(y);
        if (w == 0 || h == 0)
            throw std::runtime_error("invalid window configuration request, failed to calculate window grid");
        while (w*h > streams)
            h > w ? h-- : w--;
        while (w*h < streams)
            h > w ? w++ : h++;
        auto new_w = round(r.w / w);
        auto new_h = round(r.h / h);
        // column count, line count, cell width cell height
        return rect{ static_cast<float>(w), static_cast<float>(h), static_cast<float>(new_w), static_cast<float>(new_h) };
    }
    
    /*
    std::vector<rect> calc_grid(rect r, std::vector<Frame>& frames)
    {
        auto grid = calc_grid(r, frames.size());

        std::vector<rect> rv;
        int curr_line = -1;

        for (int i = 0; i < frames.size(); i++)
        {
            auto mod = i % (int)grid.x;
            float fw = IMU_FRAME_WIDTH;
            float fh = IMU_FRAME_HEIGHT;
            if (auto vf = frames[i].as<rs2::video_frame>())
            {
                fw = (float)vf.get_width();
                fh = (float)vf.get_height();
            }
            float cell_x_postion = (float)(mod * grid.w);
            if (mod == 0) curr_line++;
            float cell_y_position = curr_line * grid.h;
            float2 margin = { grid.w * 0.02f, grid.h * 0.02f };
            auto r = rect{ cell_x_postion + margin.x, cell_y_position + margin.y, grid.w - 2 * margin.x, grid.h };
            rv.push_back(r.adjust_ratio(float2{ fw, fh }));
        }

        return rv;
    }*/
};

// Struct for managing rotation of pointcloud view
struct glfw_state {
    glfw_state(float yaw = 15.0, float pitch = 15.0) : yaw(yaw), pitch(pitch), last_x(0.0), last_y(0.0),
        ml(false), offset_x(2.f), offset_y(2.f), tex() {}
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
    texture tex;
};

// Handles all the OpenGL calls needed to display the point cloud
#ifdef DRAW_POINTCLOUD
void draw_pointcloud(float width, float height, glfw_state& app_state,Points& points)// rs2::points& points)
{
    if (!points)
        return;

    // OpenGL commands that prep screen for the pointcloud
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
    float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
    glBegin(GL_POINTS);


    /* this segment actually prints the pointcloud */
    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    for (int i = 0; i < points.size(); i++)
    {
        if (vertices[i].z)
        {
            // upload the point and texture coordinates only for points we have depth data for
            glVertex3fv(vertices[i]);
            glTexCoord2fv(tex_coords[i]);
        }
    }

    // OpenGL cleanup
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
}
#endif
// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, glfw_state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x -= static_cast<float>(xoffset);
        app_state.offset_y -= static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}
