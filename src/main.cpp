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
/* TODO(lcf): replace crt stuff with lcf_random */

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

#define OUTPUT_IMAGE_PATH "cover.png"


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

struct Ray {
    vec3 pos;
    vec3 dir;
    vec3 dir_inv;
};

struct Material {
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

struct Hit {
    Ray normal;
    u32 mat_i;
    f32 t;
    b32 front_face;
};

struct AABB {
    vec3 min;
    vec3 max;
};

/* TODO: could be a union.. but maybe better this way? */
struct Obj {
    enum {
        SPHERE = FLAG(0),
    };
    u32 flags;
    u32 mat_i;
    vec3 pos;
    /* Sphere */
    f32 radius;
};

struct BVNode {
    AABB bound;
    union {
        u32 right_child_i; 
        u32 first_object_i;
    };
    u16 objects;
    u8 axis; /* WARN(lcf): I don't understand the reason why this is here in pbr.
                what is this used for? */
    u8 __pad;
};

struct Scene {
    BVNode *bvh;
    u32 bvs;
    Obj *object;
    u32 objects;
    Material *material;
    u32 materials;
};

b32 material_scatter
(const Material mat, const Ray r, const Hit h, Ray *scattered, vec3 *attenuation) {
    b32 out = false;
    /* TODO(lcf): refactor materials code to favor composition */
    
    if (TEST_FLAG(mat.flags, Material::DIFFUSE)) {
        scattered->pos = h.normal.pos;
        scattered->dir = rvec3_hemisphere(h.normal.dir);
        /* WARN(lcf): chance of bug with below if rvec3_unit = -normal
           need to handle if using this method.
        */
        // scattered->dir = h.normal.dir + rvec3_unit();
        *attenuation = mat.albedo;
        out = true;
    }

    if (TEST_FLAG(mat.flags, Material::METAL)) {
        vec3 reflect_dir = ReflectV3(HMM_NormV3(r.dir), h.normal.dir);
        *scattered = {h.normal.pos, reflect_dir + mat.fuzz*rvec3_unit()};
        *attenuation = mat.albedo;
        /* NOTE(lcf): this conditional says that if the scattered ray would travel into
           the surface again, cancel it's processing. This seems to be a form of
           absorbtion? */
        out = HMM_DotV3(scattered->dir, h.normal.dir) > 0.0f;
    }

    if (TEST_FLAG(mat.flags, Material::GLASS)) {
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

    scattered->dir_inv = {
        1.0f / scattered->dir.X,
        1.0f / scattered->dir.Y,
        1.0f / scattered->dir.Z
    };
    return out;
}

b32 hit_aabb(const AABB bound, const Ray r, f32 tmin, f32 tmax) {
    for (int dim = 0; dim < 3; dim++) {
        f32 t0 = (bound.min.Elements[dim] - r.pos.Elements[dim]) * r.dir_inv.Elements[dim];
        f32 t1 = (bound.max.Elements[dim] - r.pos.Elements[dim]) * r.dir_inv.Elements[dim];

        tmin = MAX(tmin, MIN(t0, t1));
        tmax = MIN(tmax, MAX(t0, t1));
    }
    return tmin < tmax;
}

b32 hit_sphere(const Obj s, const Ray r, f32 tmin, f32 tmax, Hit *h) {
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
            h->mat_i = s.mat_i;

            h->front_face = HMM_DotV3(r.dir, h->normal.dir) < 0;
            if (!h->front_face) {
                h->normal.dir = -h->normal.dir;
            }
        }
    }

    return out;
}

b32 hit_scene(const Scene s, const Ray r, f32 tmin, f32 tmax, Hit *h) {
    b32 out = false;
    
    /* Walk BVH Tree */
    /* Tree is stored as an array, with left children of i at i + 1
       For this reason a stack of right children is necessary, since they may be
       a (somewhat) arbitrary offset away from the left child.
     */
    Hit hitclosest = {};
    hitclosest.t = f32_MAX;

    /* NOTE: Stack size can be fairly small assuming tree is balanced, meaning traversal depth
       is logarithmic. Scene has up to 2^32 objects (u32), so 64 stack-slots is sufficient. */
    u32 stack[64]; u32 stackc = 0;

    u32 c = 0;
    BVNode bv;
    stack[stackc++] = c; /* push first BVNode on stack */
    for (; stackc > 0; ) {
        c = stack[--stackc];
        bv = s.bvh[c];

        if (hit_aabb(bv.bound, r, tmin, hitclosest.t)) {
            if (bv.objects > 0) {
                for (s32 i = 0; i < bv.objects; i++) {
                    Obj o = s.object[bv.first_object_i + i];
                    if (hit_sphere(o, r, tmin, hitclosest.t, &hitclosest)) {
                        out = true;
                        *h = hitclosest;
                    }
                }
            } else {
                stack[stackc++] = bv.right_child_i;
                stack[stackc++] = c + 1;
            }
        } 
    }

    return out;
}

/* TODO: consider restructuring iterations such that each pixel is iterated at each step,
   before taking more steps per pixel. This would allow previewing / early exiting from
   render. */
vec3 ray_color(Ray r, Scene s, s32 call_depth) {
    const vec3 white = {1.0f, 1.0f, 1.0f};
    const vec3 blue = {0.5f, 0.7f, 1.0f};
    const vec3 red = {1.0f, 0.0f, 0.5f};
    const vec3 black = {0.0f, 0.0f, 0.0f};

    vec3 out = {0.0f};
    
    Hit h;
    vec3 attenuation = {1.0f, 1.0f, 1.0f};
    Ray scatter;
    vec3 new_attenuation;
    for (s32 i = 0; i < call_depth; i++) {
        if (!hit_scene(s, r, 0.001f, f32_MAX, &h)) {
            vec3 unit_dir = HMM_NormV3(r.dir);
            f32 t = 0.5*(unit_dir.Y + 1.0); /* map [-1, 1] to [0, 1]; */
            out = attenuation * HMM_LerpV3(white, t, blue);
            break;
        }

        /* TODO(lcf): consider removing material_scatter abstraction and    
               directly controlling more. Can also directly accumulate attenuation.
           NOTE: that material scatter is only false for METAL materials atm, and only when
           the reflected ray would not escape the object essentially. 
        */
        /* TODO(lcf): similarly, consider inlining hit_scene */
        if (!material_scatter(s.material[h.mat_i], r, h, &scatter, &new_attenuation)) {
            out = {0.0};
            break;
        }
        
        attenuation *= new_attenuation;
        r = scatter;
    }

    return out;
}

struct camera {
    vec3 pos;
    vec3 lower_left;
    vec3 horizontal;
    vec3 vertical;
    vec3 u, v, w;
    f32 lens_radius;
};

Ray camera_ray(camera c, f32 u, f32 v) {
    vec3 rd = c.lens_radius * rvec3_unit();
    vec3 offset = c.u * rd.X + c.v * rd.Y;
    Ray out;
    out.pos = c.pos + offset;
    out.dir = c.lower_left + u*c.horizontal + v*c.vertical - c.pos - offset;
    out.dir_inv = {1.0f / out.dir.X, 1.0f / out.dir.Y, 1.0f / out.dir.Z};
    return out;
}

int main(int argc, char* argv[]) {
    os_Init();
    
    Arena *arena = Arena::create();

    const f32 aspect_ratio = 3.0f / 2.0f;
    const s32 Width = 600;
    const s32 Height = static_cast<s32>(Width / aspect_ratio);
    const s32 Channels = 3; /* RGB */
    const s32 SamplesPerPixel = 10;
    const s32 CallDepthPerPixel = 50;
    const s32 OutputScale = 3;
    const s32 OutputWidth = OutputScale*Width;
    const s32 OutputHeight = OutputScale*Height;
        
    /* Memory */
    vec3 *RenderBuffer = arena->take_array_zero<vec3>(static_cast<u32>(Width*Height));
    u8 *OutputBuffer = arena->take_array<u8>(OutputWidth*OutputHeight*Channels);

    /* Build Scene */
    printf("building scene: \n");
    Scene Scene = {0};
    {
        Scene.object = arena->take_array<Obj>(512);
        Scene.material = arena->take_array<Material>(512);

        /* Ground */
        Scene.material[Scene.materials++] = {Material::DIFFUSE, {0.5f, 0.5f, 0.5f}, 1.0f};
        Scene.object[Scene.objects++] = {Obj::SPHERE, 0, { 0.0f, -1000.0f, 0.0f}, 1000.0f};

        s32 sz = 0;
        for (s32 a = -sz; a < sz; a++) {
            for (s32 b = -sz; b < sz; b++) {
                Obj spr = {};
                f32 radius = 0.2f;
                vec3 pos = {a + 0.9f * rf32(), radius, b + 0.9f * rf32()};
                spr = {Obj::SPHERE, Scene.materials, pos, radius};
                Scene.object[Scene.objects++] = spr;
                
                Material mat = {};
                f32 choose_mat = rf32();
                if (choose_mat < 0.8f) { /* Diffuse */
                    mat.flags = Material::DIFFUSE;
                    mat.albedo = rvec3()*rvec3();
                } else if (choose_mat < 0.95f) { /* Metal */
                    mat.flags = Material::METAL;
                    mat.albedo = rvec3_range(0.5f, 1.0f);
                    mat.fuzz = rf32_range(0.0f, 0.5f);
                } else { /* Glass */
                    mat.flags = Material::GLASS;
                    mat.refraction = 1.5;
                }
                Scene.material[Scene.materials++] = mat;
            }
        }

        Scene.object[Scene.objects++] = {Obj::SPHERE, Scene.materials,
                                         {0.0f, 1.0f, 0.0f}, 1.0f};
        Scene.material[Scene.materials++] = {Material::GLASS, {}, 0.0f, 1.5f};

        Scene.object[Scene.objects++] = {Obj::SPHERE, Scene.materials,
                                         {-4.0f, 1.0f, 0.0f}, 1.0f};
        Scene.material[Scene.materials++] = {Material::DIFFUSE, {0.4f, 0.2f, 0.1f}, 0.f, 0.f};

        Scene.object[Scene.objects++] = {Obj::SPHERE, Scene.materials,
                                         {4.0f, 1.0f, 0.0f}, 1.0f};
        Scene.material[Scene.materials++] = {Material::METAL, {0.7f, 0.6f, 0.5f}, 0.f, 0.f};
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
                Ray r = camera_ray(Camera, u, v);
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
    ASSERT(stbi_write_png(OUTPUT_IMAGE_PATH, OutputWidth, OutputHeight, Channels,
                          OutputBuffer, OutputWidth*Channels) > 0);
    printf("done! >> " OUTPUT_IMAGE_PATH "\n");
    printf("Rendered in %f seconds!\n", (renderEndTime - renderStartTime) / 1.0e6);
}
