//#include <tbb/tbb.h>
#include <stdio.h>
#include <math.h>
#include <gmtl/gmtl.h>
#include <imageio.h>
#include <iostream>
#include <world.h>
#include <standard.h>
#include "../lights/pointlight.h"

#include <parsers/ass_parser.h>

#include <reporter.h>

#define WIN32_LEAN_AND_MEAN
#define AMBIENT_INT 0.01f
#include <Windows.h>
#include <algorithm>

#include <main.h>

int g_RenderMaxDepth = 12;

extern int g_pixel_samples;

Spectrum traceRay(World* world, gmtl::Rayf& ray, int depth = 0);

World* ReadFromFile(const char* filename)
{
    World* world;
    if (!ReadAssFile(filename, world))
    {
        Error("Could not read file: %s", filename);
        return NULL;
    }
    return world;
}

float roulette() {
	float r = static_cast<float>(rand());
	float n = r / static_cast<float>(RAND_MAX);

	return n;
}

float Halton(int i, int base) {
	float res = 0.0f;
	float f = 1.0f / base;
	
	while (i > 0) {
		res += f * (i % base);
		i = i / base;
		f = f / base;
	}

	return res;
}

gmtl::Vec3f refractedDirection(float ni, float nt, const gmtl::Vec3f& V, const gmtl::Vec3f& N)
{
	gmtl::Vec3f T;
	float eta;

	eta = ni / nt;
	float c1 = -dot(V, N);
	float c2_op = 1.0f - eta*eta*(1.0f - c1*c1);
	if (c2_op < 0.0f)
		return gmtl::Vec3f(0.0f);

	float c2 = sqrt(c2_op);
	T = eta*V + (eta*c1 - c2)*N;

	return T;
}

gmtl::Vec3f transmittedDirection(bool& entering, const gmtl::Vec3f& N, const gmtl::Vec3f& V, const Standard* mat)
{
	gmtl::Vec3f normal_refraction = N;
	bool exitingObject = dot(N, -V) < 0.0f;
	float ni = 0.0f;
	float nt = 0.0f;

	if (exitingObject)
	{
		ni = mat->refractionIndex;
		nt = 1.0003f; // air refraction index
		normal_refraction = -normal_refraction;
	}
	else
	{
		ni = 1.0003f; // air refraction index
		nt = mat->refractionIndex;
	}

	gmtl::Vec3f T = refractedDirection(ni, nt, V, normal_refraction);

	entering = !exitingObject;
	return T;
}

Spectrum directRadiance(World* world, Ray &ray, IntersectInfo info) 
{

	Spectrum dirLight(0.0f);
	
	if (world->mLights.size() > 0) 
	{
		int lightIndex = rand() % world->mLights.size();
		
		gmtl::Vec3f wi;
		float lPdf;
		gmtl::Rayf visibility;

		// muestra de un punto de luz aleatorio
		Spectrum lSample = world->mLights.at(lightIndex)->Sample(info.position, wi, lPdf, visibility);

		// desplazamos el origen del rayo para que no colisione con el propio objeto
		visibility.setOrigin(visibility.getOrigin() + info.normal * 0.001f);

		// comprobamos con que colisiona el rayo de visibilidad
		IntersectInfo nInfo;
		world->intersect(nInfo, visibility);

		// si interseca con algun objeto de la escena
		if (nInfo.objectID == -1)
		{
			// calculamos el BRDF
			dirLight = info.material->BRDF(lSample, wi, -ray.getDir(), info);
			dirLight = (dirLight / lPdf) / static_cast<float>(1 / world->mLights.size());
		}
	}

	return dirLight;
}

Spectrum indirectRadiance(World* world, Ray &ray, IntersectInfo info, int recursivityDepth)
{
	Spectrum totalIndLight(0.0f);
	gmtl::Vec3f wi, wo;
	float random = roulette();

	if (random < 0.1f)
	{
		// establecemos la iluminacion a 1.0f
		totalIndLight = Spectrum(1.0f);

		// calculamos la direccion del siguiente rebote
		// numero aleatorio para el angulo
		float random1 = roulette();

		// numero aleatorio para las coordenadas
		float random2 = random / static_cast<float>(RAND_MAX);

		float phi = 2 * M_PI * random1;

		float x = cos(phi) * (sqrt(1.0f - (random2 * random2)));
		float y = sin(phi) * (sqrt(1.0f - (random2 * random2)));
		float z = random2;

		// generamos el rayo con el origen desplazado para que no colisione con el propio objeto
		gmtl::Rayf indRay = gmtl::Rayf((info.position + info.normal * 0.001f), gmtl::Vec3f(x, y, z));

		// calculamos iluminacion
		totalIndLight = totalIndLight * traceRay(world, indRay, recursivityDepth + 1);

		// multiplicamos la iluminacion por el BRDF del material
		totalIndLight = totalIndLight * info.material->BRDF(totalIndLight, indRay.getDir(), -ray.getDir(), info);

		// multiplicamos por el coseno del angulo que forma el rayo con la normal
		gmtl::Vec3f dirNormalized = indRay.getDir();
		normalize(dirNormalized);
		totalIndLight = totalIndLight * max(gmtl::dot(info.normal, dirNormalized), 0.0f);

		// dividimos por el pdf
		totalIndLight = totalIndLight / info.material->pdf(wi, wo) * 0.1f;
	}

	return totalIndLight;
}

Spectrum depthOfField(World *world, Ray& ray) {
	gmtl::Vec3f dir, fPoint;
	gmtl::Point3f oriDOF;
	Spectrum colorDOF(0.0f);
	Ray rayDOF;

	// calculamos la direccion del foco con la distancia del dof a 30
	dir = ray.getDir();
	normalize(dir);
	fPoint = ray.getOrigin() + dir * 30.0f;

	// generamos rayos 
	for (int i = 1; i <= 10; i++) 
	{
		// calculamos el origen del rayo
		float x, y, z;
		gmtl::Vec3f newDir;

		x = roulette(); y = roulette(); z = roulette();
		oriDOF = ray.getOrigin() + gmtl::Vec3f(x, y, z) * 0.5f;

		// calculamos la direccion del rayo
		newDir = gmtl::Vec3f(fPoint - oriDOF);
		normalize(newDir);

		// construimos el rayo
		rayDOF.setOrigin(oriDOF + dir * 0.01f);
		rayDOF.setDir(newDir);

		colorDOF += traceRay(world, rayDOF, 1);
	}

	// hacemos el promedio
	colorDOF = colorDOF / 10.0f;

	return colorDOF;
}

Spectrum traceRay(World* world, gmtl::Rayf& ray, int depth) 
{
	IntersectInfo info;

	world->intersect(info, ray);

	Spectrum dirRadiance(0.0f);
	Spectrum indirRadiance(0.0f);

	// calculamos la radiancia directa y la indirecta del punto de colision
	if (info.objectID != InvalidObjectID)
	{
		dirRadiance = directRadiance(world, ray, info);
		indirRadiance = indirectRadiance(world, ray, info, depth);
	}

	return dirRadiance + indirRadiance;
}

void render_image(World* world, unsigned int dimX, unsigned int dimY, float* image, float* alpha)
{
	srand(time(NULL));

	for (int i = 0; i < dimX; i++) {
		for (int j = 0; j < dimY; j++) {
			gmtl::Rayf ray;
			Spectrum s = Spectrum(0.0f, 0.0f, 0.0f);
			
			// lanzamos nRays rayos por pixel
			int nRays = 5;

			for (int nRay = 0; nRay < nRays; nRay++) {
				// halton
				float h1 = Halton(nRay, 3);
				float h2 = Halton(nRay, 5);

				// rayo camara pixel
				ray = world->getCamera()->generateRay((float)i + h1, (float)j + h2);

				// iluminacion rayo
				s += traceRay(world, ray, 1);
			}

			s = s / static_cast<float>(nRays);
		
			// calculamos el DOF
			gmtl::Vec3f distVec;
			Spectrum colorDOF(0.0f);
			IntersectInfo info;
			world->intersect(info, ray);

			if (info.objectID != InvalidObjectID) {
				distVec = info.position - ray.getOrigin();
				float dist = gmtl::length(distVec);

				// calculamos si el objeto esta enfocado
				if (abs(dist - 30.0f) >= 1.0f) {
					colorDOF = depthOfField(world, ray);
				}
			}

			// sumamos el dof al color
			s += colorDOF;

			// aplicamos el color correspondiente a cada pixel
			image[((j*dimX + i) * 3)] = s[0];
			image[((j*dimX + i) * 3) + 1] = s[1];
			image[((j*dimX + i) * 3) + 2] = s[2];
			alpha[j*dimX + i] = 1.0f;
		}
	}
}

unsigned int g_intersectTriangleCalls;
extern "C"
{
    __declspec(dllexport) void renderSceneFromFile(float*& image, float*& alpha, World*& world, const char* filename)
    {
        google::InitGoogleLogging("rendering.dll");
        FLAGS_logtostderr = 1;

        g_intersectTriangleCalls = 0;

        // Create world from file
        world = ReadFromFile(filename);
        if (!world)
        {
            fprintf(stderr, "Error reading file %s. Press enter to exit", filename);
            getc(stdin);
            return;
        }
        INITREPORTER("report.ma", world);
        unsigned int dimX = world->getCamera()->getResolution()[0];
        unsigned int dimY = world->getCamera()->getResolution()[1];

        image = new float[dimX*dimY * 3];
        alpha = new float[dimX*dimY];

        // Compute pixel values
        clock_t tStart = clock();
        render_image(world, dimX, dimY, image, alpha);

        clock_t tEnd = clock();
        LOG(INFO) << "Time taken: " << (double)(tEnd - tStart) / CLOCKS_PER_SEC << "s";
        LOG(INFO) << "Triangles intersected: " << g_intersectTriangleCalls;

        google::ShutdownGoogleLogging();
    }

    __declspec(dllexport) void WriteImg(const char* name, float *pixels, float *alpha, int xRes,
        int yRes, int totalXRes, int totalYRes, int xOffset, int yOffset)
    {
        WriteImage(name, pixels, alpha, xRes, yRes, totalXRes, totalYRes, xOffset, yOffset);
    }
}

// dllmain.cpp : Defines the entry point for the DLL application.

BOOL APIENTRY DllMain(HMODULE hModule,
    DWORD  ul_reason_for_call,
    LPVOID lpReserved
)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}
