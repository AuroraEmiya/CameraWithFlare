#include "Shader.h"
char phonefs[] =
    "#version 330 core\n"
    "\n"
    "struct Material {\n"
    "    vec3 ambient;\n"
    "    vec3 diffuse;\n"
    "    vec3 specular;\n"
    "    float shininess;\n"
    "};\n"
    "\n"
    "struct Light {\n"
    "    vec3 position;\n"
    "    vec3 ambient;\n"
    "    vec3 diffuse;\n"
    "    vec3 specular;\n"
    "};\n"
    "\n"
    "in vec3 FragPos;\n"
    "in vec3 Normal;\n"
    "\n"
    "out vec4 FragColor;\n"
    "\n"
    "uniform vec3 viewPos;\n"
    "uniform Material material;\n"
    "uniform Light light;\n"
    "\n"
    "void main()\n"
    "{\n"
    "    // Ambient\n"
    "    vec3 ambient = light.ambient * material.ambient;\n"
    "\n"
    "    // Diffuse\n"
    "    vec3 norm = normalize(Normal);\n"
    "    vec3 lightDir = normalize(light.position - FragPos);\n"
    "    float diff = max(dot(norm, lightDir), 0.0);\n"
    "    vec3 diffuse = light.diffuse * (diff * material.diffuse);\n"
    "\n"
    "    // Specular\n"
    "    vec3 viewDir = normalize(viewPos - FragPos);\n"
    "    vec3 reflectDir = reflect(-lightDir, norm);\n"
    "    float spec = pow(max(dot(viewDir, reflectDir), 0.0), "
    "material.shininess);\n"
    "    vec3 specular = light.specular * (spec * material.specular);\n"
    "\n"
    "    vec3 result = ambient + diffuse + specular;\n"
    "    FragColor = vec4(result, 1.0);\n"
    "}";

char phonevs[] =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPosition;\n"
    "layout (location = 1) in vec3 aNormal;\n"
    "\n"
    "out vec3 FragPos;\n"
    "out vec3 Normal;\n"
    "\n"
    "uniform mat4 model;\n"
    "uniform mat4 view;\n"
    "uniform mat4 projection;\n"
    "\n"
    "void main()\n"
    "{\n"
    "    FragPos = vec3(model * vec4(aPosition, 1.0));\n"
    "    Normal = mat3(transpose(inverse(model))) * aNormal;\n"
    "\n"
    "    gl_Position = projection * view * vec4(FragPos, 1.0);\n"
    "}";

char screenfs[] =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "in vec2 TexCoords;\n"
    "uniform sampler2D screenTexture;\n"
    "void main()\n"
    "{\n"
    "   FragColor = texture(screenTexture, TexCoords);\n"
    "}";

char screenvs[] =
    "#version 330 core\n"
    "layout (location = 0) in vec2 aPos;\n"
    "layout (location = 1) in vec2 aTexCoords;\n"
    "out vec2 TexCoords;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);\n"
    "   TexCoords = aTexCoords;\n"
    "}";
