#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "stb_image_write.h"
#include <cmath>
#include <limits>
#include <ctime>
#include <thread>
#include <algorithm>
#include "bvh.h"

using namespace parser;

class Ray {
    public:
        Vec3f origin{};
        Vec3f direction{};
        int reflectionDepth;
        bool isShadowRay;
    
    Ray(Vec3f o, Vec3f d, bool shadow) {
        this->origin = o;
        this->direction = d;
        this->isShadowRay = false;
    }
};

class Hit {
    public:
        bool isHit;
        Vec3f hitPoint{};
        float t;
        Vec3f surfaceNormal{};
        Vec3f color{0.0f,0.0f,0.0f};
        int materialID;
};

float DotProduct(const Vec3f &a, const Vec3f &b) {
    float result = a.x * b.x + a.y * b.y + a.z * b.z;
    return result;
}

Vec3f CrossProduct(const Vec3f &a, const Vec3f &b) {
    float i = a.y * b.z - a.z * b.y;
    float j = a.x * b.z - a.z * b.x;
    float k = a.x * b.y - a.y * b.x;
    Vec3f result {i, -j, k};

    return result;
}

Vec3f NormalizeVector3f(const Vec3f &vector) {
    float distance = sqrtf(powf(vector.x,2) + powf(vector.y,2) + powf(vector.z,2));
    Vec3f result{vector.x / distance, vector.y/distance, vector.z/distance};

    return result;
}

Vec3f SubstractVectors(const Vec3f &a, const Vec3f &b) {
    Vec3f result{};
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}

Vec3f SumVectors(const Vec3f &a, const Vec3f &b) {
    Vec3f result{};
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;

    return result;
}

Vec3f NegateVector(const Vec3f &vector) {
    Vec3f result{};
    result.x = -vector.x;
    result.y = -vector.y;
    result.z = -vector.z;

    return result;
}

Vec3f MultiplyVectorWithConstant(const Vec3f &vector, float c) {
    Vec3f result{};
    result.x = c * vector.x;
    result.y = c * vector.y;
    result.z = c * vector.z;

    return result;
}

float CalculateDeterminant(const Vec3f &a, const Vec3f &b,const Vec3f &c) {
    return (a.x * b.y * c.z) - (a.x * b.z * c.y) - (a.y * b.x * c.z) + (a.y * b.z * c.x) + (a.z * b.x * c.y) - (a.z * b.y * c.x); 
}

float findMin(float x, float y) {
    if(x <= y) {
        return x;
    }
    else {
        return y;
    }
}

float findMax(float x, float y) {
    if(x >= y) {
        return x;
    }
    else {
        return y;
    }
}

void clampColor(Vec3f &color) {
    if(color.x > 255) {
        color.x = 255;
    }

    if(color.y > 255) {
        color.y = 255;
    }

    if(color.z > 255) {
        color.z = 255;
    }
}

Vec3f FindNormal(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{
    return NormalizeVector3f(CrossProduct(SubstractVectors(b,a), SubstractVectors(c,a)));
}


Ray SendRayToPixel(const Camera &cam, int i, int j) {
    
    // u = v x w
    Vec3f u = NormalizeVector3f(CrossProduct(cam.up,NegateVector(cam.gaze)));

    Vec3f m = SumVectors(cam.position, MultiplyVectorWithConstant(cam.gaze,cam.near_distance));
    Vec3f q = SumVectors(SumVectors(m, MultiplyVectorWithConstant(u,cam.near_plane.x)), MultiplyVectorWithConstant(cam.up, cam.near_plane.w));

    // near plane: left right bottom top
    float su = (cam.near_plane.y - cam.near_plane.x) * (i + 0.5f) / cam.image_width;
    float sv = (cam.near_plane.w - cam.near_plane.z) * (j + 0.5f) / cam.image_height;

    Vec3f s = SubstractVectors(SumVectors(q,MultiplyVectorWithConstant(u,su)), MultiplyVectorWithConstant(cam.up,sv));

    Ray ray {cam.position, NormalizeVector3f(SubstractVectors(s,cam.position)),false};

    return ray;
}

Hit SphereIntersection(const Ray &ray, const Sphere &sphere, const Vec3f &center) {

    Hit hit {};
    float t;

    Vec3f o_min_c = SubstractVectors(ray.origin,center);

    float A = DotProduct(ray.direction,ray.direction);
    float B = 2 * DotProduct(ray.direction,o_min_c);
    float C = DotProduct(o_min_c,o_min_c) - powf(sphere.radius,2);

    float delta = powf(B,2) - (4*A*C);

    if(delta < 0) {
        hit.isHit = false;
        return hit;
    }
    else if(delta > 0) {
        // double intersection
        float t1 = (-B + sqrt(delta)) / (2 * A);
        float t2 = (-B - sqrt(delta)) / (2 * A);

        if(t1 < 0 && t2 < 0) {
            hit.isHit = false;
            return hit;
        }
        else if(t1 > 0 && t2 > 0) {
            t = findMin(t1,t2);
        }
        else {
            t = findMax(t1,t2);
        }

        Vec3f intersection = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,t));
        hit.isHit = true;
        hit.t = t;
        hit.hitPoint = intersection;
        hit.materialID = sphere.material_id;
        hit.surfaceNormal = NormalizeVector3f(SubstractVectors(intersection,center));
    
        return hit;
    }
    else if(delta == 0) {
        // single intersection
        t = -B / (2 * A);

        Vec3f intersection = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,t));
        hit.isHit = true;
        hit.hitPoint = intersection;
        hit.t = t;
        hit.surfaceNormal = NormalizeVector3f(SubstractVectors(intersection,center));
        hit.materialID = sphere.material_id;

        return hit;
    } 

    // UNKNOWN CASES
    hit.isHit = false;
    return hit;
}

Hit TriangleIntersection(const Ray &ray, const Vec3f &a, const Vec3f &b, const Vec3f &c) {

    Hit hit{};
    hit.surfaceNormal = FindNormal(a,b,c);

    if(DotProduct(hit.surfaceNormal,ray.direction) > 0 && !ray.isShadowRay) {
        hit.isHit = false;
        return hit;
    }

    Vec3f a_minus_c = SubstractVectors(a,c);
    Vec3f b_minus_c = SubstractVectors(b,c);
    Vec3f o_minus_c = SubstractVectors(ray.origin,c);
    Vec3f minusD = NegateVector(ray.direction);


    float detDelta = CalculateDeterminant(a_minus_c,b_minus_c, minusD);

    if(detDelta == 0.0f) {
        hit.isHit = false;
        return hit;
    }

    float detAlpha = CalculateDeterminant(o_minus_c,b_minus_c,minusD);
    float detBeta = CalculateDeterminant(a_minus_c,o_minus_c,minusD);
    float detT = CalculateDeterminant(a_minus_c,b_minus_c,o_minus_c);

    float alpha = detAlpha / detDelta;
    float beta = detBeta / detDelta;
    float t = detT / detDelta;
    float gamma = 1 - alpha - beta;

    if((0 <= alpha && alpha <= 1) && (0 <= beta && beta <= 1) && (0 <= gamma && gamma <= 1) && t > 0) {
        // then this is a hit!
        Vec3f intersection = SumVectors(ray.origin, MultiplyVectorWithConstant(ray.direction, t));
        hit.isHit = true;
        hit.hitPoint = intersection;
        hit.t = t;

        return hit;
    }

    // No hit!
    hit.isHit = false;
    return hit;
}

Hit MeshInterSection(const Ray &ray, const Mesh &mesh, const Scene &scene) {
    
    Hit finalHit{};
    finalHit.isHit = false;
    finalHit.t = std::numeric_limits<float>::max();

    for(int i = 0; i < mesh.faces.size(); i ++) {

        Face currentFace = mesh.faces[i];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        Hit currentHit = TriangleIntersection(ray,v0,v1,v2);

        if(currentHit.isHit) {
            if(currentHit.t < finalHit.t) {
                if(ray.isShadowRay) {
                    finalHit = currentHit;
                    return finalHit;
                }
                currentHit.materialID = mesh.material_id;
                finalHit = currentHit;
            }
        }
    }

    return finalHit;
}

bool BVHIntersection(const Ray &ray, BVHNode* bvh, Hit &hit, const Scene &scene);

Hit FindClosestRayIntersection(const Ray &ray,const Scene &scene, std::vector<BVHNode*> bvh, bool isBVHRender) {

    Hit closestHit{};
    closestHit.isHit = false;
    closestHit.t = std::numeric_limits<float>::max();


    // send rays to the spheres
    for(int i = 0; i < scene.spheres.size(); i++) {
        Sphere currentSphere = scene.spheres[i];
        Matrix4x4 transform = currentSphere.transformationMatrix;
        transform = transform.inverse();

        Ray transformRay = ray;
        transformRay.origin = transform.MultiplicationWithPoint(ray.origin);
        transformRay.direction = transform.MultiplicationWithVector(ray.direction);

        Hit currentHit = SphereIntersection(transformRay,currentSphere,scene.vertex_data[currentSphere.center_vertex_id - 1]);
        
        if(currentHit.isHit && currentHit.t < closestHit.t) {
            currentHit.hitPoint = currentSphere.transformationMatrix.MultiplicationWithPoint(currentHit.hitPoint);
            currentHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(currentHit.surfaceNormal));
            closestHit = currentHit;
        }
    }

    // send rays to the triangles
    for(int i = 0; i < scene.triangles.size(); i++) {
        Face currentTriangle = scene.triangles[i].indices;
        Matrix4x4 transform = currentTriangle.transformationMatrix;
        transform = transform.inverse();

        Ray transformRay = ray;
        transformRay.origin = transform.MultiplicationWithPoint(ray.origin);
        transformRay.direction = transform.MultiplicationWithVector(ray.direction);

        Hit currentHit = TriangleIntersection(transformRay,scene.vertex_data[currentTriangle.v0_id - 1] , scene.vertex_data[currentTriangle.v1_id - 1], scene.vertex_data[currentTriangle.v2_id - 1]);
        currentHit.hitPoint = currentTriangle.transformationMatrix.MultiplicationWithPoint(currentHit.hitPoint);
        currentHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(currentHit.surfaceNormal));
        if(currentHit.isHit && currentHit.t < closestHit.t) {
            closestHit = currentHit;
            closestHit.materialID = scene.triangles[i].material_id;
        }
    }

    if(isBVHRender) {
        for(int i = 0; i < bvh.size(); i++) {
            if(bvh[i]) {
                Hit bvhHit;
                bvhHit.isHit = false;
                bvhHit.t = std::numeric_limits<float>::max();
                bool isBvhHit = BVHIntersection(ray,bvh[i],bvhHit,scene);
                if(isBvhHit && bvhHit.isHit && bvhHit.t < closestHit.t) {
                    closestHit = bvhHit;
                }
            }
        }
    }
    else {
        for(int i = 0; i < scene.meshes.size(); i++) {
            Mesh currentMesh = scene.meshes[i];

            Matrix4x4 transform = currentMesh.transformationMatrix;
            transform = transform.inverse();

            Hit currentHit = MeshInterSection(ray,currentMesh,scene);
            if(currentHit.isHit && currentHit.t < closestHit.t) {
                closestHit = currentHit;
                closestHit.materialID = scene.meshes[i].material_id;
            }
        }
    }


    return closestHit;
    
}

bool ShadowTest(const Ray &ray, const Hit &currentHit,const Scene &scene,const PointLight &light, std::vector<BVHNode*> bvh, bool isBVHRender) {
    // returns true if the intersection point is in shadow for current light

    Vec3f epsilonPoint = MultiplyVectorWithConstant(currentHit.surfaceNormal,scene.shadow_ray_epsilon);
    Vec3f shadowRayOrigin = SumVectors(epsilonPoint,currentHit.hitPoint);

    Vec3f shadowRayDirection = NormalizeVector3f(SubstractVectors(light.position,currentHit.hitPoint));
    float distance = sqrtf(powf(light.position.x - currentHit.hitPoint.x,2) + powf(light.position.y - currentHit.hitPoint.y,2) + powf(light.position.z - currentHit.hitPoint.z,2));

    Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};

    Hit shadowHit;
    shadowHit.isHit = false;
    shadowHit.t = std::numeric_limits<float>::max();
    //bool isShadowHit = BVHIntersection(shadowRay,bvh,shadowHit,scene);

    shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh,isBVHRender);

    if( shadowHit.isHit && shadowHit.t < distance) {
        //std::cout << shadowHit.t << "*" << distance << "\n";
        return true;
    }
    
    return false;
}

Vec3f DiffuseShading(Hit &hit,const PointLight &light, const Scene &scene) {

    Vec3f incomingRadiance{0,0,0};
    Material hitMaterial = scene.materials[hit.materialID - 1];
    float distance = sqrtf(powf(light.position.x - hit.hitPoint.x,2) + powf(light.position.y - hit.hitPoint.y,2) + powf(light.position.z - hit.hitPoint.z,2));
    float cosTheta = DotProduct(NormalizeVector3f(SubstractVectors(light.position,hit.hitPoint)),hit.surfaceNormal);

    if(cosTheta < 0.0f) {
        return incomingRadiance;
    }

    incomingRadiance.x = (light.intensity.x / powf(distance,2)) * cosTheta * hitMaterial.diffuse.x;
    incomingRadiance.y = (light.intensity.y / powf(distance,2)) * cosTheta * hitMaterial.diffuse.y;
    incomingRadiance.z = (light.intensity.z / powf(distance,2)) * cosTheta * hitMaterial.diffuse.z;

    return incomingRadiance;
}

Vec3f SpecularShading(Ray &ray, Hit &hit, const PointLight &light, const Scene &scene) {
    // blinn-phong model

    Vec3f incomingRadiance{0,0,0};
    Material hitMaterial = scene.materials[hit.materialID - 1];

    Vec3f wIncoming = NormalizeVector3f(SubstractVectors(light.position,hit.hitPoint));
    Vec3f wOutgoing = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));

    float cosTheta = DotProduct(wIncoming,hit.surfaceNormal);

    if(cosTheta < 0.0f) {
        return incomingRadiance;
    }

    Vec3f halfVector{};
    halfVector = NormalizeVector3f(SumVectors(wOutgoing,wIncoming));

    float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);
    if(cosAlpha <= 0) {cosAlpha = 0.0f;}

    float phongAlpha;
    if(hitMaterial.has_phong) {
        phongAlpha = powf(cosAlpha,hitMaterial.phong_exponent);
    }
    else {
        phongAlpha = powf(cosAlpha,1);
    }
    

    float distance = sqrtf(powf(light.position.x - hit.hitPoint.x,2) + powf(light.position.y - hit.hitPoint.y,2) + powf(light.position.z - hit.hitPoint.z,2));

    incomingRadiance.x = (light.intensity.x / powf(distance,2)) * phongAlpha * hitMaterial.specular.x;
    incomingRadiance.y = (light.intensity.y / powf(distance,2)) * phongAlpha * hitMaterial.specular.y;
    incomingRadiance.z = (light.intensity.z / powf(distance,2)) * phongAlpha * hitMaterial.specular.z;

    return incomingRadiance;
}

Vec3f AmbientShading(Hit &hit,const Scene &scene) {

    Vec3f incomingRadiance{};
    Material hitMaterial = scene.materials[hit.materialID - 1];

    incomingRadiance.x = scene.ambient_light.x * hitMaterial.ambient.x;
    incomingRadiance.y = scene.ambient_light.y * hitMaterial.ambient.y;
    incomingRadiance.z = scene.ambient_light.z * hitMaterial.ambient.z;

    return incomingRadiance;
}

Ray MirrorReflection(Ray &ray, Hit &hit, const Scene &scene) {

    Vec3f epsilonedOrigin = SumVectors(MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon),hit.hitPoint);
    Vec3f incomingDirection = NegateVector(ray.direction);
    float cosTheta = DotProduct(incomingDirection,hit.surfaceNormal);
    Vec3f reflectionDirection = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(hit.surfaceNormal,(2*cosTheta)),incomingDirection));
    Ray reflectionRay{epsilonedOrigin,reflectionDirection,false};
    return reflectionRay;
}


Vec3f ApplyShadings(Ray &ray, Hit &hit, const Scene &scene, std::vector<BVHNode*> bvh, bool isBVHRender) {

    Vec3f radiance{0.0f,0.0f,0.0f};
    Material currentMaterial = scene.materials[hit.materialID - 1];

    if(ray.reflectionDepth > scene.max_recursion_depth) {
        return radiance;
    }

    Vec3f ambient = AmbientShading(hit,scene);
    radiance.x += ambient.x;
    radiance.y += ambient.y;
    radiance.z += ambient.z;


    for(int i = 0; i < scene.point_lights.size(); i++) {
        PointLight currentLight = scene.point_lights[i];

        if(ShadowTest(ray,hit,scene,currentLight,bvh,isBVHRender) == false) {
            Vec3f diffuse = DiffuseShading(hit,currentLight,scene);
            radiance.x += diffuse.x; 
            radiance.y += diffuse.y; 
            radiance.z += diffuse.z; 

            Vec3f specular = SpecularShading(ray,hit,currentLight,scene);
            radiance.x += specular.x;
            radiance.y += specular.y;
            radiance.z += specular.z;
            
        }
            
    }

    if(currentMaterial.is_mirror && ray.reflectionDepth <= scene.max_recursion_depth) {

        Ray reflectionRay = MirrorReflection(ray,hit,scene);
        Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh,isBVHRender);
        ray.reflectionDepth += 1;
        reflectionRay.reflectionDepth = ray.reflectionDepth;
        
        if(reflectionHit.isHit) {

            Vec3f reflectionRadiance{0.0f,0.0f,0.0f};
            reflectionRadiance = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,isBVHRender);

            radiance.x += reflectionRadiance.x * currentMaterial.mirror.x;
            radiance.y += reflectionRadiance.y * currentMaterial.mirror.y;
            radiance.z += reflectionRadiance.z * currentMaterial.mirror.z;
        } 
        
    }
    

    if(currentMaterial.is_conductor && ray.reflectionDepth <= scene.max_recursion_depth) {
        float n2 = currentMaterial.refractionIndex;
        float k2 = currentMaterial.absorbtionIndex;

        float cosTheta = -DotProduct(ray.direction,hit.surfaceNormal);
        Vec3f wr = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(hit.surfaceNormal,2*cosTheta),NegateVector(ray.direction)));

        float Rs = (n2*n2 + k2*k2 - 2*n2*cosTheta + cosTheta*cosTheta) / (n2*n2 + k2*k2 + 2*n2*cosTheta + cosTheta*cosTheta);
        float Rp = ((n2*n2 + k2*k2)*cosTheta*cosTheta - 2*n2*cosTheta + 1) / ((n2*n2 + k2*k2)*cosTheta*cosTheta + 2*n2*cosTheta + 1);

        float Fr = (Rs + Rp) / 2;

        Ray reflectionRay{SumVectors(MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon),hit.hitPoint),wr,false};
        Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh,isBVHRender);
        ray.reflectionDepth += 1;
        reflectionRay.reflectionDepth = ray.reflectionDepth;
        
        if(reflectionHit.isHit) {
            Vec3f reflectionRadiance = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,isBVHRender);

            radiance.x += reflectionRadiance.x * currentMaterial.mirror.x * Fr;
            radiance.y += reflectionRadiance.y * currentMaterial.mirror.y * Fr;
            radiance.z += reflectionRadiance.z * currentMaterial.mirror.z * Fr;
        }   
    }

    if(currentMaterial.is_dielectric && ray.reflectionDepth <= scene.max_recursion_depth) {
        float n1 = 1.0f; // vacuum
        float n2 = currentMaterial.refractionIndex;

        Vec3f incomingDirection = NegateVector(ray.direction);
        Vec3f normalDirection = hit.surfaceNormal; // pointing outside
        float cosTheta = DotProduct(incomingDirection,normalDirection);
        Vec3f reflectedDirection = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(normalDirection,2*cosTheta),incomingDirection));
        Vec3f refractedDirection;

        bool rayIsEntering = cosTheta > 0.0f;
        float insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));
        float cosPhi;

        if(rayIsEntering) {
            cosPhi = sqrtf(insideTerm);
        
            Vec3f dNcosTheta = SumVectors(MultiplyVectorWithConstant(normalDirection,cosTheta),ray.direction);
            refractedDirection.x = dNcosTheta.x * (n1/n2) - normalDirection.x * cosPhi;
            refractedDirection.y = dNcosTheta.y * (n1/n2) - normalDirection.y * cosPhi;
            refractedDirection.z = dNcosTheta.z * (n1/n2) - normalDirection.z * cosPhi;

        }
        else {
            // swap n
            n1 = currentMaterial.refractionIndex;
            n2 = 1.0f; // vacuum
            
            insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));

            if(insideTerm < 0) {
                n1 = 1.0f;
                n2 = currentMaterial.refractionIndex;
                insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));
            }

            cosPhi = sqrtf(insideTerm);

            normalDirection = NegateVector(normalDirection); // pointing inside
            cosTheta = -cosTheta;
            rayIsEntering = !rayIsEntering;

            Vec3f dNcosTheta = SumVectors(MultiplyVectorWithConstant(normalDirection,cosTheta),ray.direction);
            refractedDirection.x = dNcosTheta.x * (n1/n2) - normalDirection.x * cosPhi;
            refractedDirection.y = dNcosTheta.y * (n1/n2) - normalDirection.y * cosPhi;
            refractedDirection.z = dNcosTheta.z * (n1/n2) - normalDirection.z * cosPhi;

            normalDirection = NegateVector(normalDirection); // pointing outside
        }


        float Fr, Ft;
        float rparallel, rperpendicular;

        rparallel = (n2*cosTheta - n1*cosPhi) / (n2*cosTheta + n1*cosPhi);
        rperpendicular = (n1*cosTheta - n2*cosPhi) / (n1*cosTheta + n2*cosPhi);

        Fr = (rparallel * rparallel + rperpendicular * rperpendicular) / 2;
        Ft = 1-Fr;

        if(Fr == 1.0f) {
            // full reflection

            Ray reflectionRay{SumVectors(hit.hitPoint, MultiplyVectorWithConstant(reflectedDirection,scene.shadow_ray_epsilon)),reflectedDirection,false};
            Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh,isBVHRender);
            ray.reflectionDepth++;
            reflectionRay.reflectionDepth = ray.reflectionDepth;
            Vec3f reflectionColor{0,0,0};
            if(reflectionHit.isHit) {
                reflectionColor = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,isBVHRender);
            }

            radiance.x += (reflectionColor.x * Fr);
            radiance.y += (reflectionColor.y * Fr);
            radiance.z += (reflectionColor.z * Fr);
        }
        else {
            Ray reflectionRay{SumVectors(hit.hitPoint, MultiplyVectorWithConstant(reflectedDirection,scene.shadow_ray_epsilon)),reflectedDirection,false};
            Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh,isBVHRender);
            ray.reflectionDepth++;
            reflectionRay.reflectionDepth = ray.reflectionDepth;
            Vec3f reflectionColor{0,0,0};
            if(reflectionHit.isHit) {
                reflectionColor = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,isBVHRender);
            } 
            

            Ray refractionRay{SumVectors(hit.hitPoint,MultiplyVectorWithConstant(refractedDirection,scene.shadow_ray_epsilon)),refractedDirection,false};
            Hit refractionHit = FindClosestRayIntersection(refractionRay,scene,bvh,isBVHRender);
            ray.reflectionDepth++;
            refractionRay.reflectionDepth = ray.reflectionDepth;
            Vec3f refractionColor{0,0,0};
            if(refractionHit.isHit) {
                refractionColor = ApplyShadings(refractionRay,refractionHit,scene,bvh,isBVHRender);
            }  

            float distance = sqrtf(powf(refractionHit.hitPoint.x- hit.hitPoint.x,2) + powf(refractionHit.hitPoint.y- hit.hitPoint.y,2) + powf(refractionHit.hitPoint.z- hit.hitPoint.z,2));

            Vec3f attenuation{0,0,0};

            attenuation.x = refractionColor.x * expf(-currentMaterial.absorbtionCoeff.x * distance);
            attenuation.y = refractionColor.y * expf(-currentMaterial.absorbtionCoeff.y * distance);
            attenuation.z = refractionColor.z * expf(-currentMaterial.absorbtionCoeff.z * distance);

            radiance.x += attenuation.x * Ft  + (reflectionColor.x * Fr);
            radiance.y += attenuation.y * Ft  + (reflectionColor.y * Fr);
            radiance.z += attenuation.z * Ft  + (reflectionColor.z * Fr);
            }

    }

    return radiance;
}

// BVH

BoundingBox TransformBoundingBox(const BoundingBox& box, Matrix4x4& transformationMatrix) {
    // Get the eight corners of the bounding box
    Vec3f corners[8] = {
        {box.min.x, box.min.y, box.min.z},
        {box.max.x, box.min.y, box.min.z},
        {box.min.x, box.max.y, box.min.z},
        {box.min.x, box.min.y, box.max.z},
        {box.max.x, box.max.y, box.min.z},
        {box.max.x, box.min.y, box.max.z},
        {box.min.x, box.max.y, box.max.z},
        {box.max.x, box.max.y, box.max.z}
    };

    // Transform each corner and update the bounding box
    BoundingBox transformedBox;
    transformedBox.min = transformedBox.max = transformationMatrix.MultiplicationWithPoint(corners[0]);

    for (int i = 1; i < 8; ++i) {
        Vec3f transformedCorner = transformationMatrix.MultiplicationWithPoint(corners[i]);

        // Update min and max
        transformedBox.min.x = std::min(transformedBox.min.x, transformedCorner.x);
        transformedBox.min.y = std::min(transformedBox.min.y, transformedCorner.y);
        transformedBox.min.z = std::min(transformedBox.min.z, transformedCorner.z);

        transformedBox.max.x = std::max(transformedBox.max.x, transformedCorner.x);
        transformedBox.max.y = std::max(transformedBox.max.y, transformedCorner.y);
        transformedBox.max.z = std::max(transformedBox.max.z, transformedCorner.z);
    }

    return transformedBox;
}

float RayBoxIntersection(const Ray &ray, const BoundingBox &bb) {

    float t1x = (bb.min.x - ray.origin.x) / ray.direction.x; // min
    float t2x = (bb.max.x - ray.origin.x) / ray.direction.x; // max

    if(t1x > 0 && t2x < t1x) {
        std::swap(t1x,t2x);
    }

    float t1y = (bb.min.y - ray.origin.y) / ray.direction.y; // min
    float t2y = (bb.max.y - ray.origin.y) / ray.direction.y; // max

    if(t1y > 0 && t2y < t1y) {
        std::swap(t1y,t2y);
    }

    float t1z = (bb.min.z - ray.origin.z) / ray.direction.z; // min
    float t2z = (bb.max.z - ray.origin.z) / ray.direction.z; // max

    if(t1z > 0 && t2z < t1z) {
        std::swap(t1z,t2z);
    }


    float maxt1 = std::max(t1x,t1y);
    maxt1 = std::max(maxt1,t1z);

    float mint2 = std::min(t2x,t2y);
    mint2 = std::min(mint2,t2z);

    // Check if the ray origin is inside the box
    bool originInside = 
        ray.origin.x >= bb.min.x && ray.origin.x <= bb.max.x &&
        ray.origin.y >= bb.min.y && ray.origin.y <= bb.max.y &&
        ray.origin.z >= bb.min.z && ray.origin.z <= bb.max.z;

    // If origin is inside, use only the smallest positive t value of exits (mint2)
    if (originInside && mint2 > 0) {
        return mint2;
    }
    
    // Standard outside-ray-box intersection
    if (maxt1 > mint2 || mint2 < 0) {
        return std::numeric_limits<float>::max(); // No valid intersection
    }

    return maxt1;

}

bool BVHIntersection(const Ray &ray, BVHNode* bvh, Hit &hit, const Scene &scene) {

    bool result = false;
    float t = RayBoxIntersection(ray,bvh->box);
    if(t < 0.0f || t == std::numeric_limits<float>::max()) {
        hit.isHit = false;
        return false;
    }


    if(bvh->faces.size() > 0) {
        Face currentFace = bvh->faces[0];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        Ray transformRay = ray;
        Matrix4x4 transform = currentFace.transformationMatrix;
        transform = transform.inverse();
        transformRay.direction = transform.MultiplicationWithVector(transformRay.direction);
        transformRay.origin = transform.MultiplicationWithPoint(transformRay.origin);

        Hit primitiveHit = TriangleIntersection(transformRay,v0,v1,v2);
        primitiveHit.hitPoint = currentFace.transformationMatrix.MultiplicationWithPoint(primitiveHit.hitPoint);
        primitiveHit.surfaceNormal = transform.inverse().MultiplicationWithVector(primitiveHit.surfaceNormal);

        if (primitiveHit.isHit && primitiveHit.t < hit.t) {
            //std::cout << primitiveHit.hitPoint.x << "****" <<hit.hitPoint.x << "\n";
            hit = primitiveHit;
            hit.materialID = currentFace.materialID;
            return true;
        }
    }

    if(bvh->left) {
        Hit hitleft;
        hitleft.t = std::numeric_limits<float>::max();
        bool leftresult = BVHIntersection(ray,bvh->left,hitleft,scene);
        if(leftresult && hitleft.t < hit.t && hitleft.t > 0.0f) {
            hit = hitleft;
            hit.isHit = true;
            result = true;
        }
    }
    

    if(bvh->right) {
        Hit hitright;
        hitright.t = std::numeric_limits<float>::max();
        bool rightresult = BVHIntersection(ray,bvh->right,hitright,scene);
        if(rightresult && hitright.t < hit.t && hitright.t > 0.0f) {
            hit = hitright;
            hit.isHit = true;
            result = true;
        }
    }

    return result;
}

std::vector<BoundingBox> computeBoundingBoxesForFaces(std::vector<Face> &allFaces, const Scene &scene) {
    
    std::vector<BoundingBox> bbVector;

    // Loop through the objects in the specified range and expand the bounding box
    for (int i = 0; i < allFaces.size(); i++) {

        // Initialize the bounding box with extreme values
        Vec3f minPoint{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vec3f maxPoint ={-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};

        Face currentFace = allFaces[i];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        minPoint.x = std::min(v0.x,v1.x);
        minPoint.x = std::min(minPoint.x,v2.x);
        minPoint.y = std::min(v0.y,v1.y);
        minPoint.y = std::min(minPoint.y,v2.y);
        minPoint.z = std::min(v0.z,v1.z);
        minPoint.z = std::min(minPoint.z,v2.z);

        maxPoint.x = std::max(v0.x,v1.x);
        maxPoint.x = std::max(maxPoint.x,v2.x);
        maxPoint.y = std::max(v0.y,v1.y);
        maxPoint.y = std::max(maxPoint.y,v2.y);
        maxPoint.z = std::max(v0.z,v1.z);
        maxPoint.z = std::max(maxPoint.z,v2.z);

        float x = (minPoint.x + maxPoint.x) * 0.5f;
        float y = (minPoint.y + maxPoint.y) * 0.5f;
        float z = (minPoint.z + maxPoint.z) * 0.5f;
        Vec3f center{x,y,z};    

        allFaces[i].boundingBox.min = minPoint;
        allFaces[i].boundingBox.max = maxPoint;
        allFaces[i].boundingBox.center = center;

        bbVector.push_back(allFaces[i].boundingBox);
    }

    return bbVector;
}

BVHNode* buildBVH(std::vector<Face> &allFaces, int start, int end, int splitAxis) {
    // check if there's mesh
    if(allFaces.size() == 0){
        return nullptr;
    }

    BVHNode* node = new BVHNode();

    // Compute the bounding box of all objects in this node
    BoundingBox box;
    Vec3f nodeMin{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    Vec3f nodeMax ={-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};

    for(int i = start; i < end; i++) {
        BoundingBox currentBox = allFaces[i].boundingBox;
        BoundingBox transformedBB = TransformBoundingBox(currentBox,allFaces[i].transformationMatrix);

        nodeMin.x = std::min(transformedBB.min.x,nodeMin.x);
        nodeMin.y = std::min(transformedBB.min.y,nodeMin.y);
        nodeMin.z = std::min(transformedBB.min.z,nodeMin.z);

        nodeMax.x = std::max(transformedBB.max.x,nodeMax.x);
        nodeMax.y = std::max(transformedBB.max.y,nodeMax.y);
        nodeMax.z = std::max(transformedBB.max.z,nodeMax.z);

        
    }
    box.min = nodeMin;
    box.max = nodeMax;
    node->box = box;

    // If there's only one object, make it a leaf node
    if (end - start == 1) {
        node->faces.push_back(allFaces[start]);
        return node;
    }

    // Sort objects by their centroids along one axis (e.g., x-axis)
    // split axis: 0 -> x
    // split axis: 1 -> y
    // split axis: 2 -> z
    if(splitAxis > 2) {splitAxis = 0;}

    if(splitAxis == 0) {
        std::sort(allFaces.begin() + start, allFaces.begin() + end,
              [splitAxis](Face a, Face b) {
                  return a.boundingBox.center.x < b.boundingBox.center.x;
              });
    }
    if(splitAxis == 1) {
        std::sort(allFaces.begin() + start, allFaces.begin() + end,
              [splitAxis](Face a, Face b) {
                  return a.boundingBox.center.y < b.boundingBox.center.y;
              });
    }
    if(splitAxis == 2) {
        std::sort(allFaces.begin() + start, allFaces.begin() + end,
              [splitAxis](Face a, Face b) {
                  return a.boundingBox.center.z < b.boundingBox.center.z;
              });
    }
    

    // Split objects into two groups
    int mid = (start+end) / 2;

    // Recursively build the left and right child nodes
    node->left = buildBVH(allFaces, start, mid, splitAxis+1);
    node->right = buildBVH(allFaces, mid, end, splitAxis+1);
    return node;
}

// Grid
int gridIndex(int x, int y, int z, int sizeX, int sizeY, int sizeZ)
{
    return z*sizeX*sizeY + y*sizeX + x;
}

int clampCellIndex(int number, int max) {
    if(number < 0) {
        return 0;
    }

    if(number >= max) {
        return max-1;
    }

    return number;
}

int findIndexOfCell(float pos, float minBB, float cellSize, int axisSize) {
    int index = std::floor((pos - minBB) / cellSize);
    if (index < 0) {
        return 0;
    }
    else if(index >= axisSize) {
        return axisSize - 1;
    }
    else {
        return index;
    }
}

BoundingBox computeSceneBoundingBoxGrid(const std::vector<Vec3f> &vertices)
{
    BoundingBox sceneBoundingBox;
    sceneBoundingBox.min;
    sceneBoundingBox.min.x = std::numeric_limits<float>::max();
    sceneBoundingBox.min.y = std::numeric_limits<float>::max();
    sceneBoundingBox.min.z = std::numeric_limits<float>::max();

    sceneBoundingBox.max;
    sceneBoundingBox.max.x = -std::numeric_limits<float>::max();
    sceneBoundingBox.max.y = -std::numeric_limits<float>::max();
    sceneBoundingBox.max.z = -std::numeric_limits<float>::max();

    for (int i = 0; i < vertices.size(); i++) {
        sceneBoundingBox.min.x = findMin(sceneBoundingBox.min.x, vertices[i].x);
        sceneBoundingBox.min.y = findMin(sceneBoundingBox.min.y, vertices[i].y);
        sceneBoundingBox.min.z = findMin(sceneBoundingBox.min.z, vertices[i].z);

        sceneBoundingBox.max.x = findMax(sceneBoundingBox.max.x, vertices[i].x);
        sceneBoundingBox.max.y = findMax(sceneBoundingBox.max.y, vertices[i].y);
        sceneBoundingBox.max.z = findMax(sceneBoundingBox.max.z, vertices[i].z);
    }

    return sceneBoundingBox;
}

Grid BuildGridUniformly(const Scene &scene)
{
    Grid grid;
    grid.gridBoundingBox = computeSceneBoundingBoxGrid(scene.vertex_data);

    // Cube root of number of faces for each axis
    int N = scene.allMeshFaces.size();
    int numOfCellsPerAxis = powf(N, (1/3));
    grid.sizeX = numOfCellsPerAxis;
    grid.sizeY = numOfCellsPerAxis;
    grid.sizeZ = numOfCellsPerAxis;
    grid.gridCells.resize(grid.sizeX * grid.sizeY * grid.sizeZ);

    // Uniform cell sizes
    float differenceX = grid.gridBoundingBox.max.x - grid.gridBoundingBox.min.x;
    float differenceY = grid.gridBoundingBox.max.y - grid.gridBoundingBox.min.y;
    float differenceZ = grid.gridBoundingBox.max.z - grid.gridBoundingBox.min.z;
    grid.cellSizeX = differenceX / grid.sizeX;
    grid.cellSizeY = differenceY / grid.sizeY;
    grid.cellSizeZ = differenceZ / grid.sizeZ;


    for (int i = 0; i < scene.allMeshFaces.size(); i++) {
        Face currentFace = scene.allMeshFaces[i];

        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        BoundingBox faceBoundingBox;
        float minTmp = findMin(v0.x,v1.x);
        faceBoundingBox.min.x = findMin(minTmp,v2.x);
        minTmp = findMin(v0.y,v1.y);
        faceBoundingBox.min.y = findMin(minTmp,v2.y);
        minTmp = findMin(v0.z,v1.z);
        faceBoundingBox.min.z = findMin(minTmp,v2.z);

        float maxTmp = findMax(v0.x,v1.x);
        faceBoundingBox.max.x = findMax(maxTmp,v2.x);
        maxTmp = findMax(v0.y,v1.y);
        faceBoundingBox.max.y = findMin(maxTmp,v2.y);
        maxTmp = findMax(v0.z,v1.z);
        faceBoundingBox.max.z = findMax(maxTmp,v2.z);

        BoundingBox transformedBoundingBox = TransformBoundingBox(faceBoundingBox,currentFace.transformationMatrix);

        // Find index of the cell for current face
        int indexMinX = std::floor((transformedBoundingBox.min.x - grid.gridBoundingBox.min.x) / grid.cellSizeX);
        int indexMinY = std::floor((transformedBoundingBox.min.y - grid.gridBoundingBox.min.y) / grid.cellSizeY);
        int indexMinZ = std::floor((transformedBoundingBox.min.z - grid.gridBoundingBox.min.z) / grid.cellSizeZ);

        int indexMaxX = std::floor((transformedBoundingBox.max.x - grid.gridBoundingBox.min.x) / grid.cellSizeX);
        int indexMaxY = std::floor((transformedBoundingBox.max.y - grid.gridBoundingBox.min.y) / grid.cellSizeY);
        int indexMaxZ = std::floor((transformedBoundingBox.max.z - grid.gridBoundingBox.min.z) / grid.cellSizeZ);

        indexMinX = clampCellIndex(indexMinX, grid.sizeX);
        indexMinY = clampCellIndex(indexMinY, grid.sizeY);
        indexMinZ = clampCellIndex(indexMinZ, grid.sizeZ);

        indexMaxX = clampCellIndex(indexMaxX, grid.sizeX);
        indexMaxY = clampCellIndex(indexMaxY, grid.sizeY);
        indexMaxZ = clampCellIndex(indexMaxZ, grid.sizeZ);

        // Assign face to cells
        for (int z = indexMinZ; z <= indexMaxZ; z++) {
            for (int y = indexMinY; y <= indexMaxY; y++) {
                for (int x = indexMinX; x <= indexMaxX; x++) {
                    int index = gridIndex(x, y, z, grid.sizeX, grid.sizeY, grid.sizeZ);
                    grid.gridCells[index].facesByIndex.push_back(i);
                }
            }
        }
    }


    return grid;
}

bool IsRayBoxIntersection(const Ray &ray, const BoundingBox &bb, float& enter, float& exit)
{
    enter = 0.0f;
    exit  = std::numeric_limits<float>::max();

    float t1x = (bb.min.x - ray.origin.x) / ray.direction.x; // min
    float t2x = (bb.max.x - ray.origin.x) / ray.direction.x; // max

    if(t1x > 0 && t2x < t1x) {
        std::swap(t1x,t2x);
    }

    float t1y = (bb.min.y - ray.origin.y) / ray.direction.y; // min
    float t2y = (bb.max.y - ray.origin.y) / ray.direction.y; // max

    if(t1y > 0 && t2y < t1y) {
        std::swap(t1y,t2y);
    }

    float t1z = (bb.min.z - ray.origin.z) / ray.direction.z; // min
    float t2z = (bb.max.z - ray.origin.z) / ray.direction.z; // max

    if(t1z > 0 && t2z < t1z) {
        std::swap(t1z,t2z);
    }


    float maxt1 = std::max(t1x,t1y);
    enter = std::max(maxt1,t1z);

    float mint2 = std::min(t2x,t2y);
    exit = std::min(mint2,t2z);

    if(enter > exit) {
        return false;
    }

    return true;

}

Hit RayIntersectionGrid(const Ray &ray, const Grid &grid, const Scene &scene)
{
    Hit hit;
    hit.isHit = false;
    hit.t = std::numeric_limits<float>::max();

    float tEnter, tExit;
    if (!IsRayBoxIntersection(ray, grid.gridBoundingBox, tEnter, tExit)) {
        return hit;
    }

    if (tEnter < 0){
        tEnter = 0;
    }

    Vec3f startPos = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,tEnter));

    int indexX = findIndexOfCell(startPos.x, grid.gridBoundingBox.min.x, grid.cellSizeX, grid.sizeX);
    int indexY = findIndexOfCell(startPos.y, grid.gridBoundingBox.min.y, grid.cellSizeY, grid.sizeY);
    int indexZ = findIndexOfCell(startPos.z, grid.gridBoundingBox.min.z, grid.cellSizeZ, grid.sizeZ);

    float signX, signY, signZ = 1;
    
    if(ray.direction.x >= 0) {
        signX = -1;
    }

    if(ray.direction.y >= 0) {
        signY = -1;
    }

    if(ray.direction.z >= 0) {
        signZ = -1;
    }


    float tMaxX, tMaxY, tMaxZ;
    float nextX = grid.gridBoundingBox.min.x;
    if(signX == 1) {
        nextX += indexX + grid.cellSizeX;
    }
    tMaxX = (nextX - ray.origin.x) / ray.direction.x;

    float nextY = grid.gridBoundingBox.min.y;
    if(signY == 1) {
        nextY += indexY + grid.cellSizeY;
    }
    tMaxY = (nextY - ray.origin.y) / ray.direction.y;

    float nextZ = grid.gridBoundingBox.min.z;
    if(signZ == 1) {
        nextZ += indexZ + grid.cellSizeZ;
    }
    tMaxZ = (nextZ - ray.origin.z) / ray.direction.z;


    float tDeltaX = grid.cellSizeX / abs(ray.direction.x);
    float tDeltaY = grid.cellSizeY / abs(ray.direction.y);
    float tDeltaZ = grid.cellSizeZ / abs(ray.direction.z);

    float t = tEnter;

    while (t <= tExit) {
        int cellIndex = gridIndex(indexX, indexY, indexZ, grid.sizeX, grid.sizeY, grid.sizeZ);
        std::vector<int> faceIndexes = grid.gridCells[cellIndex].facesByIndex;

        for (int i = 0; i < faceIndexes.size(); i++) {
            Face currentFace = scene.allMeshFaces[faceIndexes[i]];

            Vec3f A = scene.vertex_data[currentFace.v0_id - 1];
            Vec3f B = scene.vertex_data[currentFace.v1_id - 1];
            Vec3f C = scene.vertex_data[currentFace.v2_id - 1];

            Hit currentHit = TriangleIntersection(ray,A,B,C);
            if (currentHit.isHit && currentHit.t < hit.t) {
                hit = currentHit;
                hit.materialID = currentFace.materialID;
            }
        }
        
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                indexX += signX;
                if (indexX < 0 || indexX >= grid.sizeX){
                    break;
                }
                t = tMaxX;
                tMaxX += tDeltaX;
            } else {
                indexZ += signZ;
                if (indexZ < 0 || indexZ >= grid.sizeZ)
                {
                    break;
                }
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxY < tMaxZ) {
                indexY += signY;
                if (indexY < 0 || indexY >= grid.sizeY) {
                    break;
                }
                t = tMaxY;
                tMaxY += tDeltaY;
            } else {
                indexZ += signZ;
                if (indexZ < 0 || indexZ >= grid.sizeZ) {
                    break;
                }
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }
        if (t > tExit){
            break;
        }
    }

    return hit;
}

// k-D Tree

BoundingBox computeFaceBoundingBox(const Scene &scene, const std::vector<int> &facesByIndex) {

    BoundingBox boundingBox;

    boundingBox.min;
    boundingBox.min.x = std::numeric_limits<float>::max();
    boundingBox.min.y = std::numeric_limits<float>::max();
    boundingBox.min.z = std::numeric_limits<float>::max();

    boundingBox.max;
    boundingBox.max.x = -std::numeric_limits<float>::max();
    boundingBox.max.y = -std::numeric_limits<float>::max();
    boundingBox.max.z = -std::numeric_limits<float>::max();


    for (int i = 0; i < facesByIndex.size(); i++) {
        int faceId = facesByIndex[i];
        Face currentFace = scene.allMeshFaces[faceId - 1];

        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        float minTmp = findMin(v0.x,v1.x);
        minTmp = findMin(minTmp,v2.x);
        boundingBox.min.x = findMin(boundingBox.min.x, minTmp);

        minTmp = findMin(v0.y,v1.y);
        minTmp = findMin(minTmp,v2.y);
        boundingBox.min.y = findMin(boundingBox.min.y, minTmp);

        minTmp = findMin(v0.z,v1.z);
        minTmp = findMin(minTmp,v2.z);
        boundingBox.min.z = findMin(boundingBox.min.z, minTmp);


        float maxTmp = findMax(v0.x,v1.x);
        maxTmp = findMax(maxTmp,v2.x);
        boundingBox.max.x = findMax(boundingBox.max.x, maxTmp);

        maxTmp = findMax(v0.y,v1.y);
        maxTmp = findMax(maxTmp,v2.y);
        boundingBox.max.y = findMax(boundingBox.max.y, maxTmp);

        maxTmp = findMax(v0.z,v1.z);
        maxTmp = findMax(maxTmp,v2.z);
        boundingBox.max.z = findMax(boundingBox.max.z, maxTmp);
    }
    return boundingBox;
}

kDNode* BuildkDTree(const Scene &scene, const std::vector<int> &facesByIndex, int depth, int maxDepth, int minFaces) {

    kDNode* newNode = new kDNode();
    newNode->left = nullptr;
    newNode->right= nullptr;
    newNode->isLeaf= false;

    newNode->boundingBox = computeFaceBoundingBox(scene,facesByIndex);

    if (((facesByIndex.size() <= minFaces)) || (depth >= maxDepth)) {
        newNode->isLeaf = true;
        newNode->facesByIndex = facesByIndex;
        return newNode;
    }

    // find mid point
    int axis = depth % 3;
    float axisMin;
    float axisMax;
    
    if(axis == 0) {
        axisMin = newNode->boundingBox.min.x;
        axisMax = newNode->boundingBox.max.x;
    }
    else if(axis == 1) {
        axisMin = newNode->boundingBox.min.y;
        axisMax = newNode->boundingBox.max.y;
    }
    else if(axis == 2) {
        axisMin = newNode->boundingBox.min.z;
        axisMax = newNode->boundingBox.max.z;
    }
    float midPoint = (axisMin + axisMax) / 2;


    // find left and right using midpoint
    std::vector<int> leftFaces;
    std::vector<int> rightFaces;
    leftFaces.reserve(facesByIndex.size());
    rightFaces.reserve(facesByIndex.size());

    float minCenter = std::numeric_limits<float>::max();
    float maxCenter = -std::numeric_limits<float>::max();
    for (int i = 0; i < facesByIndex.size(); i++) {
        int faceID = facesByIndex[i];
        Face currentFace = scene.allMeshFaces[faceID - 1];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        float centerX = (v0.x + v1.x + v2.x) / 3.f;
        float centerY = (v0.y + v1.y + v2.y) / 3.f;
        float centerZ = (v0.z + v1.z + v2.z) / 3.f;

        float cAxis = (&centerX)[axis];
        if (cAxis < minCenter){
            minCenter = cAxis;
        }
        if (cAxis > maxCenter){
             maxCenter = cAxis;
        }

        if (cAxis < minCenter){
            minCenter = cAxis;
        }

        if (cAxis > maxCenter) {
            maxCenter = cAxis;
        }

        if (cAxis <= midPoint) {
            leftFaces.push_back(faceID);
        } else {
            rightFaces.push_back(faceID);
        }
    }

    // sliding
    if (leftFaces.empty() || rightFaces.empty()) {

        float slidingMidPoint = (minCenter + maxCenter) / 2;

        leftFaces.clear();
        rightFaces.clear();
        for (int i = 0; i < facesByIndex.size(); i++) {
            int faceId = facesByIndex[i];

            Face currentFace = scene.allMeshFaces[faceId - 1];
            Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
            Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
            Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

            float centerX = (v0.x + v1.x + v2.x) / 3.f;
            float centerY = (v0.y + v1.y + v2.y) / 3.f;
            float centerZ = (v0.z + v1.z + v2.z) / 3.f;

            float cAxis = (&centerX)[axis];
            if (cAxis < minCenter){
                minCenter = cAxis;
            }
            if (cAxis > maxCenter){
                maxCenter = cAxis;
            }

            if (cAxis < minCenter){
                minCenter = cAxis;
            }

            if (cAxis > maxCenter) {
                maxCenter = cAxis;
            }

            if (cAxis <= slidingMidPoint) {
                leftFaces.push_back(faceId);
            } else {
                rightFaces.push_back(faceId);
            }
        }

        if (leftFaces.empty() || rightFaces.empty()) {
            newNode->isLeaf = true;
            newNode->facesByIndex = facesByIndex;
            return newNode;
        }
    }

    // build child trees
    newNode->left  = BuildkDTree(scene, leftFaces,  depth+1, maxDepth, minFaces);
    newNode->right = BuildkDTree(scene, rightFaces, depth+1, maxDepth, minFaces);
    return newNode;
}


Hit IntersectkDTree(kDNode* node, const Ray &ray, int depth, const Scene &scene) {

    Hit hit;
    hit.isHit = false;
    hit.t = std::numeric_limits<float>::max();

    float tNear, tFar;
    if (!IsRayBoxIntersection(ray, node->boundingBox, tNear, tFar)){
        return hit;
    }

    // leaf node
    if (node->isLeaf) {
        for (int i = 0; i < node->facesByIndex.size(); i++) {

            int faceId = node->facesByIndex[i];
            Face currentFace = scene.allMeshFaces[faceId - 1];
            Vec3f A = scene.vertex_data[currentFace.v0_id - 1];
            Vec3f B = scene.vertex_data[currentFace.v1_id - 1];
            Vec3f C = scene.vertex_data[currentFace.v2_id - 1];

            Hit currentHit = TriangleIntersection(ray, A, B, C);
            if (currentHit.isHit && currentHit.t < hit.t) {
                hit = currentHit;
                hit.materialID = currentFace.materialID;
            }
        }
        return hit;
    }

    // find mid point
    int axis = depth % 3;
    float axisMin;
    float axisMax;
    
    if(axis == 0) {
        axisMin = node->boundingBox.min.x;
        axisMax = node->boundingBox.max.x;
    }
    else if(axis == 1) {
        axisMin = node->boundingBox.min.y;
        axisMax = node->boundingBox.max.y;
    }
    else if(axis == 2) {
        axisMin = node->boundingBox.min.z;
        axisMax = node->boundingBox.max.z;
    }
    float midPoint = (axisMin + axisMax) / 2;

    float originAxis;
    float dirAxis;
    if(axis == 0) {
        originAxis = ray.origin.x;
        dirAxis = ray.direction.x;
    }
    else if(axis == 1) {
        originAxis = ray.origin.y;
        dirAxis = ray.direction.y;
    }
    else if(axis == 2) {
        originAxis = ray.origin.z;
        dirAxis = ray.direction.z;
    }

    float tValue = (midPoint - originAxis) / dirAxis;
    bool leftFirst = (dirAxis < 0.f);

    kDNode* firstChild;
    kDNode* secondChild;
    if(leftFirst) {
        firstChild = node->right;
        secondChild = node->left;
    }
    else {
        firstChild = node->left;
        secondChild = node->right;
    }

    if (firstChild) {
        Hit firstHit = IntersectkDTree(firstChild, ray, depth + 1, scene);
        if (firstHit.isHit && firstHit.t < hit.t) {
            hit = firstHit;
        }
    }

    // if tValue >= 0 we check other side
    if (tValue >= 0 && secondChild) {
        if (tValue < hit.t) {
            Hit secondHit = IntersectkDTree(secondChild, ray, depth + 1, scene);
            if (secondHit.isHit && secondHit.t < hit.t) {
                hit = secondHit;
            }
        }
    }
    return hit;
}


void raytracer_default_render(int start, int end, int width, int height, unsigned char* image, Camera& camera, const parser::Scene& scene, std::vector<BVHNode*> bvh) {
    for (int y = start; y < end; ++y) {
        for (int x = 0; x < width; ++x) {
            
            int pixel_index = (y * width + x) * 3;
            Ray currentRay = SendRayToPixel(camera, x, y);
            currentRay.reflectionDepth = 0;
            Hit closestHit;
            closestHit.isHit = false;
            closestHit.t = std::numeric_limits<float>::max();

            closestHit = FindClosestRayIntersection(currentRay,scene,bvh,false);

            Vec3f color = {0,0,0};
            
            if (closestHit.isHit) {
                color = ApplyShadings(currentRay, closestHit, scene,bvh,false);
                clampColor(color);
            } else {
                color.x = scene.background_color.x;
                color.y = scene.background_color.y;
                color.z = scene.background_color.z;
            }

            image[pixel_index] = round(color.x);  // R
            image[pixel_index + 1] = round(color.y);  // G
            image[pixel_index + 2] = round(color.z);  // B
        }
    }
}

void raytracer_bvh_render(int start, int end, int width, int height, unsigned char* image, Camera& camera, const parser::Scene& scene, std::vector<BVHNode*> bvh) {
    for (int y = start; y < end; ++y) {
        for (int x = 0; x < width; ++x) {
            
            int pixel_index = (y * width + x) * 3;
            Ray currentRay = SendRayToPixel(camera, x, y);
            currentRay.reflectionDepth = 0;
            Hit closestHit;
            closestHit.isHit = false;
            closestHit.t = std::numeric_limits<float>::max();

            closestHit = FindClosestRayIntersection(currentRay,scene,bvh,true);

            Vec3f color = {0,0,0};
            
            if (closestHit.isHit) {
                color = ApplyShadings(currentRay, closestHit, scene,bvh,true);
                clampColor(color);
            } else {
                color.x = scene.background_color.x;
                color.y = scene.background_color.y;
                color.z = scene.background_color.z;
            }

            image[pixel_index] = round(color.x);  // R
            image[pixel_index + 1] = round(color.y);  // G
            image[pixel_index + 2] = round(color.z);  // B
        }
    }
}

void raytracer_grid_render(int start, int end, int width, int height, unsigned char* image, Camera& camera, const parser::Scene& scene, std::vector<BVHNode*> bvh) {
    for (int y = start; y < end; ++y) {
        for (int x = 0; x < width; ++x) {
            
            int pixel_index = (y * width + x) * 3;
            Ray currentRay = SendRayToPixel(camera, x, y);
            currentRay.reflectionDepth = 0;
            Hit closestHit;
            closestHit.isHit = false;
            closestHit.t = std::numeric_limits<float>::max();

            closestHit = RayIntersectionGrid(currentRay,scene.grid,scene);

            Vec3f color = {0,0,0};
            
            if (closestHit.isHit) {
                color = ApplyShadings(currentRay, closestHit, scene,bvh,false);
                clampColor(color);
            } else {
                color.x = scene.background_color.x;
                color.y = scene.background_color.y;
                color.z = scene.background_color.z;
            }

            image[pixel_index] = round(color.x);  // R
            image[pixel_index + 1] = round(color.y);  // G
            image[pixel_index + 2] = round(color.z);  // B
        }
    }
}

void raytracer_kdTree_render(int start, int end, int width, int height, unsigned char* image, Camera& camera, const parser::Scene& scene, std::vector<BVHNode*> bvh, kDNode* kdRoot) {
    for (int y = start; y < end; ++y) {
        for (int x = 0; x < width; ++x) {
            
            int pixel_index = (y * width + x) * 3;
            Ray currentRay = SendRayToPixel(camera, x, y);
            currentRay.reflectionDepth = 0;
            Hit closestHit;
            closestHit.isHit = false;
            closestHit.t = std::numeric_limits<float>::max();

            closestHit = IntersectkDTree(kdRoot, currentRay, 0, scene);

            Vec3f color = {0,0,0};
            
            if (closestHit.isHit) {
                color = ApplyShadings(currentRay, closestHit, scene,bvh,false);
                clampColor(color);
            } else {
                color.x = scene.background_color.x;
                color.y = scene.background_color.y;
                color.z = scene.background_color.z;
            }

            image[pixel_index] = round(color.x);  // R
            image[pixel_index + 1] = round(color.y);  // G
            image[pixel_index + 2] = round(color.z);  // B
        }
    }
}

int main(int argc, char* argv[])
{
    parser::Scene scene;


    scene.loadFromXml(argv[1]);

    int width, height;
    unsigned char* image;

    std::cout << "Rendering for : " << scene.allMeshFaces.size() << " faces\n";


    clock_t begin_bvh_build = clock();
    std::vector<BVHNode *> bvhTrees;
    for(int i = 0; i < scene.meshes.size(); i++) {
        std::vector<BoundingBox> boundingBoxForMesh = computeBoundingBoxesForFaces(scene.meshes[i].faces,scene);
        BVHNode* bvhTreeForMesh = buildBVH(scene.meshes[i].faces,0,scene.meshes[i].faces.size(),0);
        bvhTrees.push_back(bvhTreeForMesh);
    }
    clock_t end_bvh_build = clock();
    double elapsed_secs_bvh_build = double(end_bvh_build - begin_bvh_build) / CLOCKS_PER_SEC;
    std::cout << "Time elapsed for BVH Tree building: " << elapsed_secs_bvh_build << " seconds\n";

    clock_t begin_grid_build = clock();
    Grid grid = BuildGridUniformly(scene);
    scene.grid = grid;
    clock_t end_grid_build = clock();
    double elapsed_secs_grid_build = double(end_grid_build - begin_grid_build) / CLOCKS_PER_SEC;
    std::cout << "Time elapsed for Grid building: " << elapsed_secs_grid_build << " seconds\n";

    clock_t begin_kdTree_build = clock();
    std::vector<int> faceIDs;
    for (int i = 1; i <= scene.allMeshFaces.size(); i++){
        faceIDs.push_back(i);
    }
    kDNode* root = BuildkDTree(scene, faceIDs, 0, 20, 4);
    clock_t end_kdTree_build = clock();
    double elapsed_secs_kdTree_build = double(end_kdTree_build - begin_kdTree_build) / CLOCKS_PER_SEC;
    std::cout << "Time elapsed for kD Tree building: " << elapsed_secs_kdTree_build << " seconds\n";

    clock_t begin_default_render = clock();
    for(int camIterator = 0; camIterator < scene.cameras.size(); camIterator++) {

        Camera currentCam = scene.cameras[camIterator];

        width = scene.cameras[camIterator].image_width;
        height = scene.cameras[camIterator].image_height;

        image = new unsigned char [width * height * 3];

        const int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        int rows_per_thread = height / num_threads;

        for (int i = 0; i < num_threads; ++i) {
            int start_y = i * rows_per_thread;
            int end_y = (i == num_threads - 1) ? height : start_y + rows_per_thread;

            threads.push_back(std::thread(raytracer_default_render, start_y, end_y, width, height, image, std::ref(currentCam), std::ref(scene), std::ref(bvhTrees)));
        }

        for (auto& thread : threads) {
            thread.join();
        }

        clock_t end_default_render = clock();
        double elapsed_secs = double(end_default_render - begin_default_render) / CLOCKS_PER_SEC;
        std::cout << "Time elapsed for default rendering: " << elapsed_secs << " seconds\n";

        std::string output_file = "default_" + currentCam.image_name;
        stbi_write_png(output_file.c_str(), width, height, 3, image, width * 3);

        delete[] image; 
    }


    clock_t begin_bvh_render = clock();
    for(int camIterator = 0; camIterator < scene.cameras.size(); camIterator++) {

        Camera currentCam = scene.cameras[camIterator];

        width = scene.cameras[camIterator].image_width;
        height = scene.cameras[camIterator].image_height;

        image = new unsigned char [width * height * 3];

        const int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        int rows_per_thread = height / num_threads;

        for (int i = 0; i < num_threads; ++i) {
            int start_y = i * rows_per_thread;
            int end_y = (i == num_threads - 1) ? height : start_y + rows_per_thread;

            threads.push_back(std::thread(raytracer_bvh_render, start_y, end_y, width, height, image, std::ref(currentCam), std::ref(scene), std::ref(bvhTrees)));
        }

        for (auto& thread : threads) {
            thread.join();
        }

        clock_t end_bvh_render = clock();
        double elapsed_secs = double(end_bvh_render - begin_bvh_render) / CLOCKS_PER_SEC;
        std::cout << "Time elapsed for BVH rendering: " << elapsed_secs << " seconds\n";

        std::string output_file = "BVH_" + currentCam.image_name;
        stbi_write_png(output_file.c_str(), width, height, 3, image, width * 3);

        delete[] image; 
    }
    

    clock_t begin_grid_render = clock();
    for(int camIterator = 0; camIterator < scene.cameras.size(); camIterator++) {

        Camera currentCam = scene.cameras[camIterator];

        width = scene.cameras[camIterator].image_width;
        height = scene.cameras[camIterator].image_height;

        image = new unsigned char [width * height * 3];

        const int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        int rows_per_thread = height / num_threads;

        for (int i = 0; i < num_threads; ++i) {
            int start_y = i * rows_per_thread;
            int end_y = (i == num_threads - 1) ? height : start_y + rows_per_thread;

            threads.push_back(std::thread(raytracer_grid_render, start_y, end_y, width, height, image, std::ref(currentCam), std::ref(scene), std::ref(bvhTrees)));
        }

        for (auto& thread : threads) {
            thread.join();
        }

        clock_t end_grid_render = clock();
        double elapsed_secs = double(end_grid_render - begin_grid_render) / CLOCKS_PER_SEC;
        std::cout << "Time elapsed for Grid rendering: " << elapsed_secs << " seconds\n";

        std::string output_file = "grid_" + currentCam.image_name;
        stbi_write_png(output_file.c_str(), width, height, 3, image, width * 3);

        delete[] image; 
    }
    

    clock_t begin_kdTree_render = clock();
    for(int camIterator = 0; camIterator < scene.cameras.size(); camIterator++) {

        Camera currentCam = scene.cameras[camIterator];

        width = scene.cameras[camIterator].image_width;
        height = scene.cameras[camIterator].image_height;

        image = new unsigned char [width * height * 3];

        const int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        int rows_per_thread = height / num_threads;

        for (int i = 0; i < num_threads; ++i) {
            int start_y = i * rows_per_thread;
            int end_y = (i == num_threads - 1) ? height : start_y + rows_per_thread;

            threads.push_back(std::thread(raytracer_kdTree_render, start_y, end_y, width, height, image, std::ref(currentCam), std::ref(scene), std::ref(bvhTrees), std::ref(root)));
        }

        for (auto& thread : threads) {
            thread.join();
        }

        clock_t end_kdTree_render = clock();
        double elapsed_secs = double(end_kdTree_render - begin_kdTree_render) / CLOCKS_PER_SEC;
        std::cout << "Time elapsed for kD Tree rendering: " << elapsed_secs << " seconds\n";

        std::string output_file = "kd_" + currentCam.image_name;
        stbi_write_png(output_file.c_str(), width, height, 3, image, width * 3);

        delete[] image; 
    }

}
