/* (lcf) (February 05, 2023)
   Simple raytracer following
   REF: https://raytracing.github.io/books/RayTracingInOneWeekend.html

   Except using libs I like + less OOP.
 */

#include "lcf/lcf.h"
#include "lcf/lcf.c"

#include "stb/stb_image_write.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include "HandmadeMath/HandmadeMath.h"
typedef HMM_Vec3 vec3;

/* random number stuff */
/* TODO(lcf): replace this with lcf_random */

#include <stdlib.h>
inline float rf32() {
    return rand() / (RAND_MAX + 1.0f);
}

inline float rf32_range(float min, float max) {
    return min + (max-min)*rf32();
}

inline vec3 rvec3() {
    return {rf32(), rf32(), rf32()};
}

inline vec3 rvec3_range(float min, float max) {
    return {rf32_range(min,max), rf32_range(min,max), rf32_range(min,max)};
}

inline vec3 rvec3_unit() {
    while (true) {
        vec3 p = rvec3_range(-1, 1);
        if (HMM_LenSqrV3(p) <= 1.0) {
            return HMM_NormV3(p);
        }
    }
}

inline vec3 rvec3_hemisphere(const vec3 normal) {
    vec3 unit = rvec3_unit();
    return (HMM_DotV3(unit, normal) > 0.0f)? unit : -unit;
}

/* ***** */

#define OUTPUT_IMAGE_PATH "traced.png"


vec3 ReflectV3(const vec3 v, const vec3 n) {
    return v - 2.0f*HMM_DotV3(v, n)*n;
}

vec3 RefractV3(const vec3 uv, const vec3 n, float refractive_ratio) {
    float cos_o = MIN(HMM_DotV3(-uv, n), 1.0f);
    /* Get reflected components of uv perpendicular and parallel to n. */
    vec3 r_T = refractive_ratio * (uv + cos_o*n);
    vec3 r_II = -sqrt(fabs(1.0 - HMM_LenSqrV3(r_T))) * n;
    return r_T + r_II;
}

struct ray {
    vec3 pos;
    vec3 dir;
};

struct material {
    enum {
        DIFFUSE = FLAG(0),
        METAL = FLAG(1),
        GLASS = FLAG(2),
    };
    u32 flags;
    vec3 albedo;
    float fuzz; /* Reflection fuzziness */
    float refraction;
};

struct hit {
    ray normal;
    u32 mat_id;
    f32 t;
    b32 front_face;
};

struct sphere {
    vec3 pos;
    f32 radius;
    u32 mat_id;
};

struct scene {
    sphere *object;
    u32 objects;
    material *material;
    u32 materials;
};

b32 material_scatter
(const material mat, const ray r, const hit h, ray *scattered, vec3 *attenuation) {
    b32 out = false;
    
    if (TEST_FLAG(mat.flags, material::DIFFUSE)) {
        scattered->pos = h.normal.pos;
        scattered->dir = rvec3_hemisphere(h.normal.dir);
        /* WARN(lcf): chance of bug with below if rvec3_unit = -normal
           need to handle if using this method.
        */
        // scattered->dir = h.normal.dir + rvec3_unit();
        *attenuation = mat.albedo;
        out = true;
    }

    if (TEST_FLAG(mat.flags, material::METAL)) {
        vec3 reflect_dir = ReflectV3(HMM_NormV3(r.dir), h.normal.dir);
        *scattered = {h.normal.pos, reflect_dir + mat.fuzz*rvec3_unit()};
        *attenuation = mat.albedo;
        out = HMM_DotV3(scattered->dir, h.normal.dir) > 0.0f;
    }

    if (TEST_FLAG(mat.flags, material::GLASS)) {
        f32 refraction_ratio = h.front_face? (1.0f / mat.refraction) : mat.refraction;
        vec3 unit = HMM_NormV3(r.dir);

        f32 cos_o = MIN(HMM_DotV3(-unit, h.normal.dir), 1.0f);
        f32 sin_o = sqrt(1.0f - cos_o*cos_o);

        b32 can_refract = refraction_ratio * sin_o <= 1.0f;

        /* Reflectance */
        f32 reflectance = 0.0f;
        { /* Schlick's Approximation */
            f32 r0 = (1.0f - refraction_ratio) / (1.0f + refraction_ratio);
            r0 = r0*r0;
            reflectance = r0 + (1.0f - r0)*pow(1.0f - cos_o, 5.0f);
        }
        
        vec3 dir;
        if (can_refract && reflectance < rf32()) {
            dir = RefractV3(unit, h.normal.dir, refraction_ratio);
        } else {
            dir = ReflectV3(unit, h.normal.dir);
        }
        *scattered = {h.normal.pos, dir};
        *attenuation = {1.0f, 1.0f, 1.0f};
        out = true;
    }
    
    return out;
}

b32 hit_sphere(const sphere s, const ray r, f32 tmin, f32 tmax, hit *h) {
    b32 out = false;
    vec3 oc = r.pos - s.pos;
    f32 a = HMM_LenSqrV3(r.dir);
    f32 b = HMM_DotV3(oc, r.dir); /* NOTE(lcf): technically b/2 in quadr. formula. */
    f32 c = HMM_LenSqrV3(oc) - s.radius*s.radius;
    f32 disc = b*b - a*c;
    
    if (disc > 0.0f) {
        f32 sdisc = sqrt(disc);
        f32 root = (-b -sdisc)/a;
        if (root < tmin || root > tmax) {
            root = (-b +sdisc)/a;
        }
        if (tmin < root && root < tmax) {
            out = true;
            h->t = root;
            h->normal.pos = r.pos + root*r.dir;
            h->normal.dir = (h->normal.pos - s.pos) / s.radius;
            h->mat_id = s.mat_id;

            h->front_face = HMM_DotV3(r.dir, h->normal.dir) < 0;
            if (!h->front_face) {
                h->normal.dir = -h->normal.dir;
            }
        }
    }

    return out;
}

b32 hit_scene(const scene s, const ray r, f32 tmin, f32 tmax, hit *h) {
    b32 out = false;
    hit temp;
    f32 tclosest = f32_MAX;

    for (u32 i = 0; i < s.objects; i++) {
        if (hit_sphere(s.object[i], r, tmin, tclosest, &temp)) {
            out = true;
            tclosest = temp.t;
            *h = temp;
        }
    }

    return out;
}

vec3 ray_color(ray r, scene s, s32 call_depth) {
    const vec3 white = {1.0f, 1.0f, 1.0f};
    const vec3 blue = {0.5f, 0.7f, 1.0f};
    const vec3 red = {1.0f, 0.0f, 0.5f};
    const vec3 black = {0.0f, 0.0f, 0.0f};
    
    if (call_depth <= 0) {
        return black;
    }

    hit h;
    if (hit_scene(s, r, 0.001f, f32_MAX, &h)) {
        ray scatter;
        vec3 attenuation;
        if (material_scatter(s.material[h.mat_id], r, h, &scatter, &attenuation)) {
            return attenuation * ray_color(scatter, s, call_depth-1);
        }
        return black;
    }

    vec3 unit_dir = HMM_NormV3(r.dir);
    f32 t = 0.5*(unit_dir.Y + 1.0); /* map [-1, 1] to [0, 1]; */
    return HMM_LerpV3(white, t, blue);
}

struct camera {
    vec3 pos;
    vec3 lower_left;
    vec3 horizontal;
    vec3 vertical;
    vec3 u, v, w;
    f32 lens_radius;
};

ray camera_ray(camera c, f32 u, f32 v) {
    vec3 rd = c.lens_radius * rvec3_unit();
    vec3 offset = c.u * rd.X + c.v * rd.Y;
    ray out;
    out.pos = c.pos + offset;
    out.dir = c.lower_left + u*c.horizontal + v*c.vertical - c.pos - offset;
    return out;
}

int main(int argc, char* argv[]) {
    Arena *arena = Arena::create();

    const f32 aspect_ratio = 3.0f / 2.0f;
    const s32 Width = 1200;
    const s32 Height = static_cast<s32>(Width / aspect_ratio);
    const s32 Channels = 3; /* RGB */
    const s32 SamplesPerPixel = 100;
    const s32 CallDepthPerPixel = 50;
    const s32 OutputScale = 4;
    const s32 OutputWidth = OutputScale*Width;
    const s32 OutputHeight = OutputScale*Height;
        
    /* Memory */
    vec3 *RenderBuffer = arena->take_array_zero<vec3>(static_cast<u32>(Width*Height));
    u8 *OutputBuffer = arena->take_array<u8>(OutputWidth*OutputHeight*Channels);

    /* Build Scene */
    printf("building scene: \n");
    scene Scene = {0};
    {
        Scene.object = arena->take_array<sphere>(512);
        Scene.material = arena->take_array<material>(512);

        /* Ground */
        Scene.material[Scene.materials++] = {material::DIFFUSE, {0.8f, 0.8f, 0.0f}, 1.0f};
        Scene.object[Scene.objects++] = {{ 0.0f, -1000.0f, 0.0f}, 1000.0f, 0};

        s32 sz = 3;
        for (s32 a = -sz; a < sz; a++) {
            for (s32 b = -sz; b < sz; b++) {
                sphere spr = {};
                f32 radius = 0.2f;
                vec3 pos = {a + 0.9f * rf32(), radius, b + 0.9f * rf32()};
                spr = {pos, radius, Scene.materials};
                Scene.object[Scene.objects++] = spr;
                
                material mat = {};
                f32 choose_mat = rf32();
                if (choose_mat < 0.8f) { /* Diffuse */
                    mat.flags = material::DIFFUSE;
                    mat.albedo = rvec3()*rvec3();
                } else if (choose_mat < 0.95f) { /* Metal */
                    mat.flags = material::METAL;
                    mat.albedo = rvec3_range(0.5f, 1.0f);
                    mat.fuzz = rf32_range(0.0f, 0.5f);
                } else { /* Glass */
                    mat.flags = material::GLASS;
                    mat.refraction = 1.5;
                }
                Scene.material[Scene.materials++] = mat;
            }
        }
    }

    /* Build Camera */
    camera Camera;
    {
        /* Parameters */
        vec3 lookfrom = {13.0f, 2.0f, 3.0f};
        vec3 lookat = {0.0f, 0.0f, 0.0f};
        vec3 up = {0.0f, 1.0f, 0.0f};
        f32 focus_dist = 10.0f;
        f32 aperture = 0.1f;
        f32 fov = HMM_DegToRad*(20.0f);

        /* Build Camera */
        f32 h = tan(fov/2.0f);
        f32 view_height = 2.0f*h;
        f32 view_width = view_height * aspect_ratio;
        f32 focal_len = 1.0f;

        Camera.lens_radius = aperture / 2;
        
        Camera.w = HMM_NormV3(lookfrom - lookat);
        Camera.u = HMM_NormV3(HMM_Cross(up, Camera.w));
        Camera.v = HMM_Cross(Camera.w, Camera.u);

        Camera.pos = lookfrom;
        Camera.horizontal = focus_dist * view_width * Camera.u;
        Camera.vertical = focus_dist * view_height * Camera.v;
        Camera.lower_left = lookfrom - 0.5*(Camera.horizontal + Camera.vertical) - focus_dist*Camera.w;
    }

    printf("starting render... \n");
    u64 renderStartTime = os_GetTimeMicroseconds();
    /* Render: Cast Rays */
    for (s32 j = 0; j < Height; ++j) {
        for (s32 i = 0; i < Width; ++i) {
            vec3 pixel_color = {};
            f32 scale = 1.0f / static_cast<f32>(SamplesPerPixel);

            for (s32 s = 0; s < SamplesPerPixel; ++s) {
                f32 u = (i + rf32()) / (Width  - 1);
                f32 v = (j + rf32()) / (Height - 1);
                ray r = camera_ray(Camera, u, v);
                pixel_color += scale*ray_color(r, Scene, CallDepthPerPixel);
            }

            /* Gamma Correction */
            pixel_color.R = sqrt(pixel_color.R); 
            pixel_color.G = sqrt(pixel_color.G);
            pixel_color.B = sqrt(pixel_color.B);
            
            RenderBuffer[i + (Height-1-j)*Width] = pixel_color;
        }
    }

    u64 renderEndTime = os_GetTimeMicroseconds();

    /* Output + Upscale */
    u8 *write = OutputBuffer;
    for (s32 j = 0; j < OutputHeight; ++j) {
        for (s32 i = 0; i < OutputWidth; ++i) {
            vec3 col = RenderBuffer[(i / OutputScale) + (j / OutputScale)*Width];
            *(write++) = static_cast<u8>(HMM_Clamp(0.0f, col.R, 1.0f)*u8_MAX);
            *(write++) = static_cast<u8>(HMM_Clamp(0.0f, col.G, 1.0f)*u8_MAX);
            *(write++) = static_cast<u8>(HMM_Clamp(0.0f, col.B, 1.0f)*u8_MAX);
        }
    }
    stbi_write_png(OUTPUT_IMAGE_PATH, OutputWidth, OutputHeight, Channels,
                   OutputBuffer, OutputWidth*Channels);
    printf("done! >> " OUTPUT_IMAGE_PATH "\n");
    printf("Rendered in %llu microseconds!\n", renderEndTime - renderStartTime);
}
