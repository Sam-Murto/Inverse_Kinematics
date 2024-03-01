#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>	
#include <glm/gtx/string_cast.hpp>

class Bone_Animation
{
public:
	struct Arc;
	struct Node;
	struct Node_Data;
	struct Transform_Data;

public:
	Bone_Animation();
	~Bone_Animation();

	void init();
	void update(float delta_time);
	void reset();
	void traverse();
	void traverse(Arc* arc_ptr, glm::mat4 mat);
	void create_tree();
	void add_node(Node* parent, glm::vec4 color, glm::vec3 scale, glm::vec3 rotation);
	glm::mat4 create_t_matrix(glm::vec3 scale, glm::vec3 rotation);
	void delete_tree();
	void delete_tree(Arc* arc_ptr);
	void compute_jacobian();
	float compute_step_size();
	void change_dofs(glm::vec3 deltaE);
	void print_jacobian();

public:

	// Here the head of each vector is the root bone
	std::vector<glm::vec3> scale_vector;
	std::vector<glm::vec3> rotation_degree_vector;
	std::vector<glm::vec4> colors;
	std::vector<Transform_Data*> bone_transforms;
	std::vector<glm::mat4> traversal_stack;

	glm::vec3 end_effector;
	glm::vec3 goal;

	glm::vec3 jacobian[9];

	glm::vec3 root_position;
	Node* root_node;

	bool animation_enabled = false;

};

struct Bone_Animation::Arc {
public:
	Arc(glm::mat4 l_matrix, glm::mat4 a_matrix, Node* node_ptr) {
		this->l_matrix = l_matrix;
		this->a_matrix = a_matrix;
		this->node_ptr = node_ptr;
		this->arc_ptr = NULL;
	}

public:
	glm::mat4 l_matrix;
	glm::mat4 a_matrix;
	Node* node_ptr;
	Arc* arc_ptr;


};

struct Bone_Animation::Node {
public:
	Node(glm::mat4 t_matrix, Node_Data* data_ptr) {
		this->t_matrix = t_matrix;
		this->data_ptr = data_ptr;
		this->arc_ptr = NULL;
	}


public:
	glm::mat4 t_matrix;
	Node_Data* data_ptr;
	Arc* arc_ptr;

};

struct Bone_Animation::Node_Data {
public:
	Node_Data(glm::vec4 color, glm::vec3 scale, glm::vec3 rotation) {
		this->color = color;
		this->scale = scale;
		this->rotation = rotation;
	}

public:
	glm::vec3 rotation;
	glm::vec3 scale;
	glm::vec4 color;
	glm::vec4 axisX;
	glm::vec4 axisY;
	glm::vec4 axisZ;
	glm::vec4 r;

};

struct Bone_Animation::Transform_Data {
public:
	Transform_Data(glm::mat4 mat, Node_Data* data_ptr, glm::vec3 end_effector) {
		this->mat = mat;
		this->data_ptr = new Node_Data(data_ptr->color, data_ptr->scale, data_ptr->rotation);
		this->end_effector = end_effector;
	}
public:
	glm::mat4 mat;
	Node_Data* data_ptr;
	glm::vec3 end_effector;
	glm::vec3 r;
	glm::vec3 axisX;
	glm::vec3 axisY;
	glm::vec3 axisZ;

};

