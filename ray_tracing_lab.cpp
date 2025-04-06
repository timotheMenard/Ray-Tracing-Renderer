#include <stdio.h>
#include <algorithm>
#include <list>
#include <math.h>
#include <cmath>
#include <vector>
#include <string.h>
#include <iostream> 

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define M_PI  3.14159265358979323846

double sqr(double x) {return x*x;}

// Vector class: Represents a 3D vector with x, y, z components
class Vector
{
public:
    explicit Vector(double x = 0., double y = 0., double z = 0.){
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    double norm2() const {
        return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
    }
    double norm() const {
        return sqrt(norm2());
    }
    void normalize() {
        double n = norm();
        data[0] /= n;
        data[1] /= n;
        data[2] /= n;
    }
    double operator[](int i) const { return data[i]; }
    double& operator[](int i) { return data[i]; }
    double data[3];
};

// Operators for the Vector class
Vector operator+(const Vector &a, const Vector &b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector &a, const Vector &b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
    return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(const Vector &a, const Vector &b) {
    return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector &a, const Vector &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector &a, const Vector &b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

// Ray class: Represents a ray with origin O and direction u
class Ray {
public:
    Ray(Vector O, Vector u) : O(O), u(u) {}
    Vector O;
    Vector u;
};

// Intersection class: Stores information about a ray-object intersection
class Intersection {
public:
    Intersection() {}
    Intersection(double distance, Vector center, Vector P, Vector N, Vector albedo, bool isInter = true, bool isMirror = true, bool isLight = false, double refractI = 1.) : isInter(isInter), isMirror(isMirror), isLight(isLight), refractI(refractI), distance(distance), center(center), P(P), N(N), albedo(albedo) {}

    bool isInter = false;
    bool isMirror = false;
    bool isLight = false;
    double refractI = 1.;
    double distance;
    Vector center;
    Vector P;
    Vector N;
    Vector albedo;

};

// Abstract geometry class: Base class for all objects in the scene
class Geometry {
public:
    virtual Intersection intersect(const Ray& r) const = 0;

    bool isMirror;
    Vector albedo;
};

// Sphere class: Represents a sphere in the scene
class Sphere : public Geometry {
public:
    Sphere(const Vector& C, double R, const Vector& A, bool isMirror = false, double refractI = 1., bool invN = false, bool isLight = false) : C(C), R(R), refractI(refractI), invN(invN), isLight(isLight) {
        albedo = A;
        isMirror = isMirror;
    }

    Intersection intersect(const Ray &r) const override {
        bool interBool = false;
        double distance = 0.;
        Vector P, N;

        double delta = sqr(dot(r.u, r.O - C)) - ((r.O - C).norm2() - sqr(R));

        if (delta >= 0) {
            interBool = true;

            double t1 = dot(r.u, C - r.O) - sqrt(delta);
            double t2 = dot(r.u, C - r.O) + sqrt(delta);

            if (t2 < 0)
                interBool = false;
            else
                distance = (t1 >= 0) ? t1 : t2;
        }

        P = r.O + r.u * distance;
        N = P - C;
        N.normalize();
        return Intersection(distance, C, P, N, albedo, interBool, isMirror, isLight, refractI);
    }

    Vector C;
    double R;
    double refractI;
    bool invN;
    bool isLight;
};

// Generate a random direction in the hemisphere around normal N
// Using cosine-weighted distribution for physically based rendering
Vector randomCos(const Vector &N) {
    double r1, r2;
    r1 = ((double)rand() / (RAND_MAX));
    r2 = ((double)rand() / (RAND_MAX));

    double x, y, z;
    x = cos(2 * M_PI * r1) * sqrt(1 - r2);
    y = sin(2 * M_PI * r1) * sqrt(1 - r2);
    z = sqrt(r2);

    Vector T1,T2;
    double min_num = N[0];
    if (N[1]<min_num){
        min_num = N[1];
    }
    if (N[2]<min_num){
        min_num = N[2];
    }
    if (min_num == N[0]){
        T1 = Vector(0, N[2], -N[1]);
    }
    if (min_num == N[1]){
        T1 = Vector(N[1], 0, -N[0]);
    }
    if (min_num == N[2]){
        T1 = Vector(N[1], -N[0], 0);
    }
    T2 = cross(N, T1);

    T1.normalize();
    T2.normalize();
    return x*T1 + y*T2 + z*N;
}

// Scene class: Contains all objects and handles ray tracing
class Scene {
public:
    ~Scene()
    {
        for (int i = 0; i < 6; i++)
        {
            delete background[i];
        }
    }

    explicit Scene(
        const Vector &lP,
        double lightR,
        double lightI) : lightR(lightR), lightI(lightI)
    {
        // Create 6 large spheres to act as room walls
        Sphere *roomObject[6] = {
            new Sphere(Vector(-1000, 0, 0), 940, Vector (0.5, 0.8, 0.1)),
            new Sphere(Vector(1000, 0, 0), 940, Vector (0.9, 0.2, 0.3)),
            new Sphere(Vector(0, 1000, 0), 940, Vector (0.3, 0.5, 0.3)),
            new Sphere(Vector(0, -1000, 0), 990, Vector (0.6, 0.5, 0.7)),
            new Sphere(Vector(0, 0, -1000), 940, Vector (0.1, 0.6, 0.7)),
            new Sphere(Vector(0, 0, 1000), 940, Vector (0.8, 0.2, 0.9))};

        for (int i = 0; i < 6; i++) {
            background[i] = roomObject[i];
            geometries.push_back(roomObject[i]);
        }

        lightP = lP;
    }

    void addGeometry(Geometry *geometry) {
        geometries.push_back(geometry);
    }

    Intersection intersect(const Ray &r){
        Intersection out = Intersection();
        double mt = HUGE_VALF;

        for (auto &geometry : geometries) {
            Intersection intersection = geometry->intersect(r);
            if (intersection.isInter && intersection.distance < mt) {
                mt = intersection.distance;
                out = intersection;
            }
        }
        return out;
    }

    Vector getColor(const Ray &r, int rD)
    {
        if (rD < 0) { return Vector(0., 0., 0.); }
        
        Intersection I = intersect(r);
        Vector out(0., 0., 0.);

        if (I.isInter)
        {
            const double epsilon = 1e-5;
            Vector N = I.N;
            Vector P = I.P + N * epsilon;
            double ndru = dot(N, r.u);

            if (I.isMirror) {
                Ray reflectionR = Ray(P, r.u - (2 * ndru) * N);
                return getColor(reflectionR, rD - 1);
            } else if (I.refractI != 1.) {
                double n1, n2;
                if (ndru > 0) {
                    N = (-1.) * N;
                    ndru = dot(N, r.u);
                    n1 = I.refractI;
                    n2 = 1.;
                } else {
                    n1 = 1.;
                    n2 = I.refractI;
                }
                
                // Fresnel equations for reflection/refraction
                double k0 = sqr((n1 - n2) / (n1 + n2));
                P = I.P - N * epsilon;
                double d = 1. - sqr((n1 / n2)) * (1 - sqr(ndru));

                if (d > 0) {
                    Vector w = ((n1 / n2) * (r.u - ndru * N)) + ((-1.) * N * sqrt(d));

                    // Schlick's approximation for Fresnel effect
                    if (((double)rand() / (RAND_MAX)) < k0 + (1 - k0) * pow(1 - abs(dot(N, w)), 5.)) {
                        Ray reflectionR = Ray(P, r.u - (2 * ndru) * N);
                        return getColor(reflectionR, rD - 1);
                    }
                }
            } else {
                Vector xLP = I.center - lightP;
                xLP.normalize();

                Vector xprime = lightR * randomCos(xLP) + lightP;
                Vector Nprime = xprime - lightP;
                Nprime.normalize();

                Vector ds = xprime - P;
                double d = ds.norm();
                Vector omega = xprime - P;
                omega.normalize();

                Intersection LI = intersect(Ray(P, omega));
                double visibility;
                if (LI.isInter && LI.distance <= d) { 
                    visibility = 0.; } else { visibility = 1.; }
                
                // Direct light contribution + indirect light (Monte Carlo path tracing)
                out = (lightI / sqr(2 * M_PI * lightR) * I.albedo / M_PI * visibility * std::max(dot(N, omega), 0.) * std::max(dot(Nprime, (-1.) * omega), 0.) / (ds.norm2() * (dot(Nprime, xLP) / (M_PI * lightR * lightR)))) + (I.albedo * getColor(Ray(P, randomCos(N)), rD - 1));
            }
        }

        return out;
    }

    std::vector<Geometry *> geometries;
    Vector lightP;
    double lightR;
    double lightI;
    Sphere *background[6];

};

// Generate normally distributed random numbers using Box-Muller transform
void boxMuller(double stdev, double &x, double &y) {
    double r1, r2;
    r1 = ((double)rand() / (RAND_MAX));
    r2 = ((double)rand() / (RAND_MAX));
    x = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2) * stdev;
    y = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2) * stdev;
}

// Bounding Box class: Used for acceleration structure
class BoundingBox {
public:
    BoundingBox() {}
    BoundingBox(Vector minB, Vector maxB) : minB(minB), maxB(maxB) {}

    Vector diagonal() const { return maxB - minB; }
    Vector midDiag() const { return (maxB + minB) / 2.; }

    bool intersect(const Ray& r, double &t) const
    {
        double tx1, ty1, tz1;
        double tx2, ty2, tz2;
        double txmin, txmax, tymin, tymax, tzmin, tzmax;

        Vector m = minB - r.O;
        Vector M = maxB - r.O;
        Vector u = r.u;

        tx1 = m[0] / r.u[0];
        tx2 = M[0] / r.u[0];
        txmin = std::min(tx1, tx2);
        txmax = std::max(tx1, tx2);

        ty1 = m[1] / r.u[1];
        ty2 = M[1] / r.u[1];
        tymin = std::min(ty1, ty2);
        tymax = std::max(ty1, ty2);

        tz1 = m[2] / r.u[2];
        tz2 = M[2] / r.u[2];
        tzmin = std::min(tz1, tz2);
        tzmax = std::max(tz1, tz2);

        double actualT = std::max(txmin, std::max(tymin, tzmin));
        double furthestT = std::min(txmax, std::min(tymax, tzmax));

        if (furthestT>0 && furthestT > actualT) {
            t = actualT;
            return true;
        }
        return false;
    }

    Vector minB;
    Vector maxB;
};

// Triangle indices structure for OBJ file loading
class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};

// Node class for Bounding Volume Hierarchy
class Node {
public:
    ~Node() {
        if (hasChildren()) {
            delete left;
            delete right;
        }
    }
    Node() {}

    void setBoundingBox(BoundingBox inBB) { BB = inBB; }

    void setTriangles(int ST, int ET) {
        startTri = ST;
        endTri = ET;
    }

    void setChildren(Node *LC, Node *RC) {
        left = LC;
        right = RC;
    }

    bool hasChildren() const { return left != nullptr; }

    Node *retLC() { return left; }

    Node *retRC() { return right; }

    BoundingBox retBB() { return BB; }

    int retST() const { return startTri; }

    int retET() const { return endTri; }

    Node *left = nullptr;
    Node *right = nullptr;
    BoundingBox BB;
    int startTri;
    int endTri;
};

// Triangle Mesh class: Loads and renders 3D models from OBJ files
class TriangleMesh : public Geometry {
public:
    ~TriangleMesh() {}
    TriangleMesh(double S, const Vector &Of, const Vector &Al, bool Ref = false) : S(S)
    {
        root = new Node();
        isMirror = Ref;
        albedo = Al;
        offset = Of;
    }

    void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				} else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						} else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					} else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							} else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								} else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);
        for (Vector &vert : vertices){
            vert = vert * S;
            vert = vert + offset;
        }
        BVH(root, 0, indices.size());

	}

    void BVH(Node *N, int ST, int ET) {
        BoundingBox BB = computeBoundingBox(ST, ET);
        N->setBoundingBox(BB);
        N->setTriangles(ST, ET);

        int n = 0;
        double M = abs(BB.diagonal()[0]);
        for (int i = 1; i < 3; i++) {
            if (abs(BB.diagonal()[i]) > M) {
                M = abs(BB.diagonal()[i]);
                n = i;
            }
        }

        int PI = ST;
        for (int i = ST; i < ET; i++) {
            Vector b = (vertices[indices[i].vtxi] + vertices[indices[i].vtxj] + vertices[indices[i].vtxk]) / 3.;
            if (b[n] < BB.midDiag()[n]) {
                std::swap(indices[i], indices[PI]);
                PI++;
            }
        }

        if (PI <= ST || PI >= ET - 5 || ET - ST < 5) { return; }

        N->setChildren(new Node(), new Node());
        BVH(N->retLC(), ST, PI);
        BVH(N->retRC(), PI, ET);
    }

    BoundingBox computeBoundingBox(int ST, int ET) {
        double mx = HUGE_VALF, my = HUGE_VALF, mz = HUGE_VALF;
        double xM = -HUGE_VALF, yM = -HUGE_VALF, zM = -HUGE_VALF;
        for (int i = ST; i < ET; i++) {   
            std::vector<Vector> oVert;
            oVert.push_back(vertices[indices[i].vtxi]);
            oVert.push_back(vertices[indices[i].vtxj]);
            oVert.push_back(vertices[indices[i].vtxk]);
            for (auto const &vert : oVert) {
                
                if (vert[0] < mx)
                    mx = vert[0];
                else if (vert[0] > xM)
                    xM = vert[0];
                if (vert[1] < my)
                    my = vert[1];
                else if (vert[1] > yM)
                    yM = vert[1];
                if (vert[2] < mz)
                    mz = vert[2];
                else if (vert[2] > zM)
                    zM = vert[2];
            }
        }
        return BoundingBox(Vector(mx, my, mz), Vector(xM, yM, zM));
    }

    Intersection intersect(const Ray &r) const override {
        Intersection out = Intersection();
        double t, mt;
        mt = HUGE_VALF;

        if (!root->retBB().intersect(r, t)) { return out; }

        std::list<Node *> nVisit;
        nVisit.push_front(root);
        while (!nVisit.empty()) {
            Node *CN = nVisit.back();
            nVisit.pop_back();
            Node *LC = CN->retLC();
            Node *RC = CN->retRC();

            if (CN->hasChildren()) {
                if (LC->retBB().intersect(r, t) && t < mt) {
                    nVisit.push_back(LC);
                }
                if (RC->retBB().intersect(r, t) && t < mt) {
                    nVisit.push_back(RC);
                }
            } else {
                const double epsilon = 1e-5;
                Vector A, e1, e2;
                for (int i = CN->retST(); i < CN->retET(); i++) {
                    TriangleIndices triangle = indices[i];
                    A = vertices[triangle.vtxi];
                    e1 = vertices[triangle.vtxj] - A;
                    e2 = vertices[triangle.vtxk] - A;

                    double beta = dot(e2, cross(A - r.O, r.u)) / dot(r.u, cross(e1, e2));
                    double gamma = -dot(e1, cross(A - r.O, r.u)) / dot(r.u, cross(e1, e2));
                    double alpha = 1. - beta - gamma;

                    if (alpha > 0. && beta > 0. && gamma > 0.) {

                        double t = dot(A - r.O, cross(e1, e2)) / dot(r.u, cross(e1, e2));

                        if (epsilon < t && t < mt) {
                            mt = t;
                            out = Intersection(t, Vector(0, 0, 0), A + beta * e1 + gamma * e2, cross(e1, e2), albedo, true, isMirror);
                        }
                    }
                }
            }
        }
        return out;
    }

    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
    Node *root;
    double S;
    Vector offset;

};