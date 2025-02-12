#include "parser.h"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <cassert>
#include <iostream>
#include "happly.h"

void parser::Scene::readPlyFile(const std::string& filepath, const std::string& plyFile, Mesh& mesh, std::vector<parser::Vec3f> vertexData) {
    int pos = filepath.find_last_of("/");
	std::string plyDir(filepath.substr(0, pos + 1));
	std::string plyPath = plyDir + plyFile;


	happly::PLYData plyData(plyPath);

	std::vector<std::array<double, 3>> vertexPositions = plyData.getVertexPositions();
	std::vector<std::vector<int>> faceIndices = plyData.getFaceIndices<int>();

	int numberOfVertices = vertex_data.size();

    for (const auto& pos : vertexPositions) {
        Vec3f vertex;
        vertex.x = pos[0];
        vertex.y = pos[1];
        vertex.z = pos[2];

        vertex_data.push_back(vertex); 
    }

    for (const auto& face : faceIndices) {
        Face triangle;
        triangle.v0_id = numberOfVertices + face[0] + 1;
        triangle.v1_id = numberOfVertices + face[1] + 1;
        triangle.v2_id = numberOfVertices + face[2] + 1;
        triangle.materialID = mesh.material_id;
        triangle.transformationMatrix = mesh.transformationMatrix; // transformation

        allMeshFaces.push_back(triangle);
        mesh.faces.push_back(triangle);
    }
    meshes.push_back(mesh);
    mesh.faces.clear();
}


void parser::Scene::loadFromXml(const std::string &filepath)
{
    tinyxml2::XMLDocument file;
    std::stringstream stream;

    auto res = file.LoadFile(filepath.c_str());
    if (res)
    {
        throw std::runtime_error("Error: The xml file cannot be loaded.");
    }

    auto root = file.FirstChild();
    if (!root)
    {
        throw std::runtime_error("Error: Root is not found.");
    }

    //Get BackgroundColor
    auto element = root->FirstChildElement("BackgroundColor");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0 0 0" << std::endl;
    }
    stream >> background_color.x >> background_color.y >> background_color.z;


    //Get ShadowRayEpsilon
    element = root->FirstChildElement("ShadowRayEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> shadow_ray_epsilon;

    //Get IntersectionTestEpsilon
    element = root->FirstChildElement("IntersectionTestEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> intersection_test_epsilon;

    //Get MaxRecursionDepth
    element = root->FirstChildElement("MaxRecursionDepth");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0" << std::endl;
    }
    stream >> max_recursion_depth;

    

    //Get Cameras
    element = root->FirstChildElement("Cameras");
    element = element->FirstChildElement("Camera");
    Camera camera;
    while (element)
    {
        auto type = element->Attribute("type");
        if(!type) {
            auto child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Gaze");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Up");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("NearPlane");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("NearDistance");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("ImageResolution");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("ImageName");
            stream << child->GetText() << std::endl;

            stream >> camera.position.x >> camera.position.y >> camera.position.z;
            stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
            stream >> camera.up.x >> camera.up.y >> camera.up.z;
            stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
            stream >> camera.near_distance;
            stream >> camera.image_width >> camera.image_height;
            stream >> camera.image_name;

            cameras.push_back(camera);
            element = element->NextSiblingElement("Camera");
        }
        else if(strcmp(type,"lookAt") == 0) {
            auto child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("GazePoint");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Up");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("FovY");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("NearDistance");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("ImageResolution");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("ImageName");
            stream << child->GetText() << std::endl;

            stream >> camera.position.x >> camera.position.y >> camera.position.z;
            stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
            stream >> camera.up.x >> camera.up.y >> camera.up.z;
            stream >> camera.fovy;
            stream >> camera.near_distance;
            stream >> camera.image_width >> camera.image_height;
            stream >> camera.image_name;

            // gaze point to gaze vector
            camera.gaze.x = camera.gaze.x - camera.position.x;
            camera.gaze.y = camera.gaze.y - camera.position.y;
            camera.gaze.z = camera.gaze.z - camera.position.z;
            float distance = sqrtf(powf(camera.gaze.x,2) + powf(camera.gaze.y,2) + powf(camera.gaze.z,2));
            camera.gaze.x = camera.gaze.x / distance;
            camera.gaze.y = camera.gaze.y / distance;
            camera.gaze.z = camera.gaze.z / distance;

            camera.fovy = camera.fovy * (3.14f / 180.0f); // Convert FOV from degrees to radians
            float aspectRatio = static_cast<float>(camera.image_width) / static_cast<float>(camera.image_height);
            // Calculate the boundaries of the image plane 
            // left rigth bottom top
            camera.near_plane.w = camera.near_distance * std::tan(camera.fovy / 2.0f);
            camera.near_plane.z = -camera.near_plane.w;
            camera.near_plane.y = camera.near_plane.w * aspectRatio;
            camera.near_plane.x = -camera.near_plane.y;

            cameras.push_back(camera);
            element = element->NextSiblingElement("Camera");
        }
        
    }


    //Get Lights
    element = root->FirstChildElement("Lights");
    auto child = element->FirstChildElement("AmbientLight");
    stream << child->GetText() << std::endl;
    stream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
    element = element->FirstChildElement("PointLight");
    PointLight point_light;
    while (element)
    {
        child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Intensity");
        stream << child->GetText() << std::endl;

        stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;
        stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

        point_lights.push_back(point_light);
        element = element->NextSiblingElement("PointLight");
    }

    //Get Materials
    element = root->FirstChildElement("Materials");
    element = element->FirstChildElement("Material");
    Material material;
    while (element)
    {
        material.is_mirror = (element->Attribute("type", "mirror") != NULL);

        material.is_conductor = (element->Attribute("type", "conductor") != NULL);
        material.is_dielectric = (element->Attribute("type", "dielectric") != NULL);

        child = element->FirstChildElement("AmbientReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("DiffuseReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("SpecularReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("MirrorReflectance");

        
        if (child)
        {
            if(material.is_mirror || material.is_conductor) {
                stream << child->GetText() << std::endl;
            }
            
        }

        

        material.has_phong = (element->FirstChildElement("PhongExponent") != NULL);
        if(material.has_phong) {
            child = element->FirstChildElement("PhongExponent");
            stream << child->GetText() << std::endl;
        }

        // refraction and absorbtion
        if(material.is_conductor || material.is_dielectric) {
            child = element->FirstChildElement("RefractionIndex");
            if (child)
            {
                stream << child->GetText() << std::endl;
            }
        }
        

        if(material.is_conductor) {
            child = element->FirstChildElement("AbsorptionIndex");
            if (child)
            {
                stream << child->GetText() << std::endl;
            }
        }
        

        if(material.is_dielectric) {
            child = element->FirstChildElement("AbsorptionCoefficient"); 
            if (child)
            {
                stream << child->GetText() << std::endl;
            }
        }
    

        stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;
        stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;
        stream >> material.specular.x >> material.specular.y >> material.specular.z;
        if (material.is_mirror || material.is_conductor)
        {
            stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
        }

        if(material.has_phong) {
            stream >> material.phong_exponent;
        }

        if(material.is_conductor) {
            stream >> material.refractionIndex;
            stream >> material.absorbtionIndex;
        }

        if(material.is_dielectric) {
            stream >> material.refractionIndex;
            stream >> material.absorbtionCoeff.x >> material.absorbtionCoeff.y >> material.absorbtionCoeff.z;
        }
        
        
        materials.push_back(material);
        element = element->NextSiblingElement("Material");
    }
    stream.clear();

    //Get Scalings
    element = root->FirstChildElement("Transformations");
    if(element) {
        child = element->FirstChildElement("Scaling");
        Matrix4x4 scalingMatrix;
        while (child)
        {
            float sx,sy,sz;
            stream << child->GetText() << std::endl;
            stream >> sx >> sy >> sz;

            scalingMatrix = scalingMatrix.ScalingMatrix(sx,sy,sz);
            scalings.push_back(scalingMatrix);
            child = child->NextSiblingElement("Scaling");
        }
    }
    stream.clear();

    //Get Translation
    element = root->FirstChildElement("Transformations");
    if(element) {
        child = element->FirstChildElement("Translation");
        Matrix4x4 translationMatrix;
        while (child)
        {
            float tx,ty,tz;
            stream << child->GetText() << std::endl;
            stream >> tx >> ty >> tz;

            translationMatrix = translationMatrix.TranslationMatrix(tx,ty,tz);
            translations.push_back(translationMatrix);
            child = child->NextSiblingElement("Translation");
        }
    }
    stream.clear();

    //Get Rotation
    element = root->FirstChildElement("Transformations");
    if(element) {
        child = element->FirstChildElement("Rotation");
        Matrix4x4 rotationMatrix;
        while (child)
        {
            float theta,rx,ry,rz;
            stream << child->GetText() << std::endl;
            stream >> theta >> rx >> ry >> rz;

            rotationMatrix = rotationMatrix.RotationMatrix(theta,rx,ry,rz);
            rotations.push_back(rotationMatrix);
            child = child->NextSiblingElement("Rotation");
        }
    }
    stream.clear();


    //Get VertexData
    element = root->FirstChildElement("VertexData");
    stream << element->GetText() << std::endl;
    Vec3f vertex;
    while (!(stream >> vertex.x).eof())
    {
        stream >> vertex.y >> vertex.z;
        vertex_data.push_back(vertex);
    }
    stream.clear();


    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Mesh");
    Mesh mesh;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> mesh.material_id;

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        mesh.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Faces");

        auto plyFile = child->Attribute("plyFile");
		if (!plyFile) {
            stream << child->GetText() << std::endl;
            Face face;
            face.transformationMatrix = mesh.transformationMatrix; // for transformation
            while (!(stream >> face.v0_id).eof())
            {
                stream >> face.v1_id >> face.v2_id;
                face.materialID = mesh.material_id; // for BVH
                allMeshFaces.push_back(face); // for BVH
                mesh.faces.push_back(face);
            }
            stream.clear();

            meshes.push_back(mesh);
            mesh.faces.clear();
            element = element->NextSiblingElement("Mesh");
        }
        else {

            // add object pushback
            readPlyFile(filepath,plyFile,mesh,vertex_data);
            stream.clear();
            mesh.faces.clear();
            element = element->NextSiblingElement("Mesh");
        }
        
        
    }
    stream.clear();

    //Get MeshInstances
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("MeshInstance");
    Mesh meshInstance;
    while (element)
    {
        bool hasSpecialMaterial = false;
        if(element->FirstChildElement("Material")) {
            child = element->FirstChildElement("Material");
            stream << child->GetText() << std::endl;
            stream >> meshInstance.material_id;
            hasSpecialMaterial = true;
        }
        
        int id = atoi(element->Attribute("id")) - 1;

        int baseMeshID = atoi(element->Attribute("baseMeshId"));
        Mesh baseMesh = meshes[baseMeshID - 1];

        if(hasSpecialMaterial == false) {
            meshInstance.material_id = baseMesh.material_id;
        }

        const char* isResetTransform = "false";
        if(element->Attribute("resetTransform")) {
            isResetTransform = element->Attribute("resetTransform");
        }

        
        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;

        if(strcmp(isResetTransform,"true") != 0) {
            compositeMatrix = baseMesh.transformationMatrix;
        }

        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        meshInstance.transformationMatrix = compositeMatrix;
        stream.clear();

        meshInstance.faces = baseMesh.faces;

        // update face transformations, materialid
        for(int i = 0; i < meshInstance.faces.size(); i++) {
            Face currentFace = meshInstance.faces[i];
            currentFace.materialID = meshInstance.material_id;
            currentFace.transformationMatrix = meshInstance.transformationMatrix;
            
            meshInstance.faces[i] = currentFace;
        }

        meshes.insert(meshes.begin() + id, meshInstance);
        meshInstance.faces.clear();
        stream.clear();
        element = element->NextSiblingElement("MeshInstance");
    }
    stream.clear();

    //Get Triangles
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Triangle");
    Triangle triangle;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> triangle.material_id;

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        triangle.transformationMatrix = compositeMatrix;
        triangle.indices.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Indices");
        stream << child->GetText() << std::endl;
        stream >> triangle.indices.v0_id >> triangle.indices.v1_id >> triangle.indices.v2_id;

        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");
    }

    

    //Get Spheres
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Sphere");
    Sphere sphere;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> sphere.material_id;

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        sphere.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Center");
        stream << child->GetText() << std::endl;
        stream >> sphere.center_vertex_id;

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> sphere.radius;

        spheres.push_back(sphere);
        element = element->NextSiblingElement("Sphere");
    }


}
