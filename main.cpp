#define _CRT_SECURE_NO_WARNINGS 1
#include "ray_tracing_lab.cpp"

int main() {
    int W = 512;
	int H = 512;
    int nb_paths = 64; 
    double fov = 60 * M_PI / 180;

    Scene scene = Scene(Vector(-10, 20, 40), 3., 1e5);

    std::vector<unsigned char> image(512 * 512 * 3, 0);

    TriangleMesh cat = TriangleMesh(0.6, Vector(0, -10, 0), Vector(0.3, 0.2, 0.25));

    cat.readOBJ("cat.obj");
    scene.addGeometry(&cat);

    Vector Q = Vector(0, 0, 55);

    
    /*for (int i = 0; i < H; i++) { // no antialiasing
        for (int j = 0; j < W; j++) {
            Vector color(0.,0.,0.);
            Vector u(j - W/2 + 0.5, H/2 - i - 0.5, -W / (2 * tan( fov / 2)));
            u.normalize();

            Ray ray = Ray(Q, u);
            color = color +  scene.getColor(ray, 5);

            image[(i * W + j) * 3 + 0] = std::max(std::min(255., pow(color[0], 1 / 2.2) * 255), 0.);
            image[(i * W + j) * 3 + 1] = std::max(std::min(255., pow(color[1], 1 / 2.2) * 255), 0.);
            image[(i * W + j) * 3 + 2] = std::max(std::min(255., pow(color[2], 1 / 2.2) * 255), 0.);
        }
    }*/

    for (int i = 0; i < H; i++) { //with antialiasing
		for (int j = 0; j < W; j++) {
            Vector color(0.,0.,0.);
            for (int k = 0; k < nb_paths; k++){

                double x, y;
                boxMuller(1, x, y);
                x += i;
                y += j;
                Vector u(y - W/2 + 0.5, H/2 - x - 0.5, -W / (2 * tan( fov / 2)));
                u.normalize();
                Ray ray(Q,u);
                color = color + scene.getColor(ray,5);   
            }

            image[(i * W + j) * 3 + 0] = std::max(std::min(255., pow(color[0]/nb_paths, 1 / 2.2) * 255), 0.);
            image[(i * W + j) * 3 + 1] = std::max(std::min(255., pow(color[1]/nb_paths, 1 / 2.2) * 255), 0.);
            image[(i * W + j) * 3 + 2] = std::max(std::min(255., pow(color[2]/nb_paths, 1 / 2.2) * 255), 0.);
        
        }
	}

    stbi_write_png("cat.png", 512, 512, 3, &image[0], 0);

    return 0;
}