#version 330 core

// Atributos de fragmentos recebidos como entrada ("in") pelo Fragment Shader.
// Neste exemplo, este atributo foi gerado pelo rasterizador como a
// interpolação da cor de cada vértice, definidas em "shader_vertex.glsl" e
// "main.cpp".
in vec4 position_world;
in vec4 normal;

// Posição do vértice atual no sistema de coordenadas local do modelo.
in vec4 position_model;

// Coordenadas de textura obtidas do arquivo OBJ (se existirem!)
in vec2 texcoords;

in float lambert_vertex;
in float phong_vertex;

// Matrizes computadas no código C++ e enviadas para a GPU
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Identificador que define qual objeto está sendo desenhado no momento
#define SPHERE 0
#define BUNNY  1
#define PLANE  2
#define BOX    3
#define M4     4
#define FIRE   5
#define WALL   6
uniform int object_id;


// Parâmetros da axis-aligned bounding box (AABB) do modelo
uniform vec4 bbox_min;
uniform vec4 bbox_max;

// Variáveis para acesso das imagens de textura
uniform sampler2D TextureImage0;
uniform sampler2D TextureImage1;
uniform sampler2D TextureImage2;
uniform sampler2D TextureM4;
uniform sampler2D TextureFire;
uniform sampler2D TextureWall;


uniform vec4 camera_position;
// Atributos uteis a lanterna
uniform vec4 camera_view_vector;

// O valor de saída ("out") de um Fragment Shader é a cor final do fragmento.
out vec3 color;

// Constantes
#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923


void main()
{

    // Obtemos a posição da câmera utilizando a inversa da matriz que define o
    // sistema de coordenadas da câmera.
    vec4 origin = vec4(0.0, 0.0, 0.0, 1.0);
    // vec4 camera_position = inverse(view) * origin;
    vec4 camera_position = inverse(view) * origin;

    float spotlight_inner_angle = radians(45.0);
    float spotlight_outer_angle = radians(50.0);

    vec4 spotlight_position = vec4(0.0, 1.0, 0.0, 1.0);
    vec4 spotlight_orientation = vec4(0.0, -1.0, 0.0, 0.0);


    // O fragmento atual é coberto por um ponto que percente à superfície de um
    // dos objetos virtuais da cena. Este ponto, p, possui uma posição no
    // sistema de coordenadas global (World coordinates). Esta posição é obtida
    // através da interpolação, feita pelo rasterizador, da posição de cada
    // vértice.
    vec4 p = position_world;

    // Normal do fragmento atual, interpolada pelo rasterizador a partir das
    // normais de cada vértice.
    vec4 n = normalize(normal);

    // Vetor que define o sentido da fonte de luz em relação ao ponto atual.
    vec4 l = normalize((spotlight_position - p) - spotlight_orientation);

    // Vetor que define o sentido da câmera em relação ao ponto atual.
    vec4 v = normalize(camera_position - p);

    // Vetor que define o sentido da reflexão especular ideal.
    vec4 r = -l + 2*n*dot(n, l);

    // Parâmetros que definem as propriedades espectrais da superfície
    vec3 Kd;  // Refletância difusa
    vec3 Ks;  // Refletância especular
    vec3 Ka;  // Refletância ambiente
    vec3 Kd0; // Refletância difusa
    float q;  // Expoente especular para o modelo de iluminação de Phong
    float q_; // Expoente especular para o modelo de iluminação de Blinn Phong

    // Coordenadas de textura U e V
    float U = 0.0;
    float V = 0.0;

    bool useBlinnPhong = false;
    bool useGouradShading = false;

    if ( object_id == PLANE )
    {
        U = fract(p.z / 2);
        V = fract(p.x / 2);

        // Propriedades espectrais do plano
        Kd = texture(TextureImage1, vec2(U,V)).rgb;
        Ks = vec3(0.0,0.0,0.0);
        Ka = Kd/10;
        q = 1.0;

    }
    else if ( object_id == BOX )
    {
        useGouradShading = true;

        // Propriedades espectrais do zumbi
        Kd = vec3(0.5,0.5,0.5);
        Ks = vec3(0.3, 0.3, 0.3);
        Ka = Kd/100;
        q = 20.0;
    }
    else if ( object_id == M4 )
    {
        useBlinnPhong = true;

        U = texcoords.x;
        V = texcoords.y;

        // Propriedades espectrais da arma
        Kd = texture(TextureM4, vec2(U,V)).rgb;
        Ks = vec3(0.3, 0.3, 0.3);
        Ka = Kd/10;
        q = 20.0;
        q_ = 15.0;
    }
    else if ( object_id == WALL )
    {
        if (p.z >= 20.0f || p.z <= -20.0f) {
            U = fract(p.x / 2);
            V = fract(p.y / 2);
        } else {
            U = fract(p.z / 2);
            V = fract(p.y / 2);
        }

        // Propriedades espectrais da parede
        Kd = texture(TextureImage2, vec2(U,V)).rgb;
        Ks = vec3(0.3, 0.3, 0.3);
        Ka = Kd/10;
        q = 1.0;
    }
    else if ( object_id == FIRE )
    {
        // Propriedades espectrais do coelho
        Kd = vec3(1.0,0.5,0.0);
        Ks = vec3 (0.8, 0.8, 0.8);
        Ka = Kd;
        q  = 5.0;
    }


    // Espectro da fonte de iluminação
    vec3 I = vec3(1.0, 1.0, 1.0); // PREENCHA AQUI o espectro da fonte de luz

    // Espectro da luz ambiente
    vec3 Ia = vec3 (0.6, 0.6, 0.6); // PREENCHA AQUI o espectro da luz ambiente


    // Termo difuso utilizando a lei dos cossenos de Lambert
    vec3 lambert_diffuse_term = Kd*I*max(0, dot(l, n)); // PREENCHA AQUI o termo difuso de Lambert
    vec3 lambert_diffuse_term_gourad_shading = Kd*I*lambert_vertex;

    // Termo ambiente
    vec3 ambient_term = Ka*Ia; // PREENCHA AQUI o termo ambiente

    // Termo especular utilizando o modelo de iluminação de Phong
    vec3 phong_specular_term  = Ks*I*pow(max(0, dot(r, v)), q); // PREENCH AQUI o termo especular de Phong
    vec3 phong_specular_term_gourad_shading = Ks*I*pow(phong_vertex, q);

    vec4 h = (v + l)/length(v + l);
    vec3 blinn_phong_specular_term = Ks*I*pow(max(0, dot(n, h)), q_);

    // Angulo atual do raio de luz da lanterna em relacao a um vetor central
    float theta = dot(normalize(p-spotlight_position), normalize(spotlight_orientation));

    // Computa atenuacao a ser usada
    float distance_to_light = length(spotlight_position - position_world);
    float attenuation_factor = 0.3f;
    float attenuation = 1.0f / (1.0f + (attenuation_factor * pow(distance_to_light, 2)));

    // Computa graduacao da intensidade da lanterna
    float epsilon = spotlight_outer_angle - spotlight_inner_angle;
    float intensity = clamp( (theta - spotlight_outer_angle) / epsilon, 0.0f, 1.0f);



    // Spotlight test
    if(theta > cos(spotlight_outer_angle)){
        if (useBlinnPhong)
            phong_specular_term = blinn_phong_specular_term;
        if (useGouradShading) {
            lambert_diffuse_term = lambert_diffuse_term_gourad_shading;
            phong_specular_term  = phong_specular_term_gourad_shading;
        }
        color =  10 * attenuation * intensity *(lambert_diffuse_term + phong_specular_term) + ambient_term;
    } else {
        color = ambient_term;
    }

    // Cor final com correção gamma, considerando monitor sRGB.
    // Veja https://en.wikipedia.org/w/index.php?title=Gamma_correction&oldid=751281772#Windows.2C_Mac.2C_sRGB_and_TV.2Fvideo_standard_gammas
    color = pow(color, vec3(1.0,1.0,1.0)/2.2);
}
