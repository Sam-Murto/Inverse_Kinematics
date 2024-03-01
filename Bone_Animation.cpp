#include "Bone_Animation.h"
#include "Renderer.h"

Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
	root_position = { 2.0f,0.5f,2.0f };
	goal = { 3.0f, 8.0f, 3.0f };

	scale_vector =
	{
		{1.0f,1.0f,1.0f},
		{0.5f,4.0f,0.5f},
		{0.5f,3.0f,0.5f},
		{0.5f,2.0f,0.5f}
	};

	rotation_degree_vector = 
	{
		{0.0f,0.0f,0.0f},
		{0.0f,0.0f,30.0f},
		{0.0f,0.0f,30.0f},
		{0.0f,0.0f,30.0f}
	};

	colors = 
	{
		{0.7f,0.0f,0.0f,1.0f},
		{0.7f,0.7f,0.0f,1.0f},
		{0.7f,0.0f,0.7f,1.0f},
		{0.0f,0.7f,0.7f,1.0f}
	};

	create_tree();

}

void Bone_Animation::update(float delta_time)
{
	delete_tree();

	create_tree();
	traverse();

	end_effector = bone_transforms.back()->end_effector;

	if (animation_enabled && glm::distance(goal, end_effector) > 0.000001f) {

		compute_jacobian();

		//Compute step size
		float beta = compute_step_size();
		//float beta = 0.01f;
		std::cout << glm::distance(goal, end_effector) << '\n';
		glm::vec3 deltaE = beta * (goal - end_effector);


		change_dofs(deltaE);
	}



}

void Bone_Animation::reset()
{

	for (int i = 1; i < rotation_degree_vector.size(); i++)
		rotation_degree_vector[i] = glm::vec3(0.0f, 0.0f, 30.0f);
	

}

void Bone_Animation::compute_jacobian() {

	//Exclude root
	for (int i = 1; i < bone_transforms.size(); i++)
	{
		glm::mat4 parent_mat = bone_transforms[i - 1]->mat;

		//Compute r' : pivot point is at the top of the parent
		glm::vec3 r = bone_transforms[i - 1]->end_effector;

		//Undo scaling so it doesn't mess with axis calculation
		parent_mat = glm::scale(parent_mat, 1.0f / scale_vector[i - 1]);

		//Compute a' for each DOF

		glm::mat4 rX = glm::rotate(glm::mat4(1.0f), rotation_degree_vector[i].x, {1.0f, 0.0f, 0.0f});
		glm::mat4 rZ = glm::rotate(glm::mat4(1.0f), rotation_degree_vector[i].z, {0.0f, 0.0f, 1.0f});;

		glm::vec3 axisY = glm::normalize(parent_mat * rX * rZ * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));
		glm::vec3 axisZ = glm::normalize(parent_mat * rX * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f));
		glm::vec3 axisX = glm::normalize(parent_mat * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f));

		//Compute jacobian entries
		glm::vec3 jX = glm::cross(axisX, (end_effector - r));
		glm::vec3 jY = glm::cross(axisY, (end_effector - r));
		glm::vec3 jZ = glm::cross(axisZ, (end_effector - r));

		//Put in jacobian matrix
		jacobian[i * 3 - 3] = jX;
		jacobian[i * 3 - 2] = jY;
		jacobian[i * 3 - 1] = jZ;

	}

}

float Bone_Animation::compute_step_size() {

	// Step size is || Jt * (g - e) || ^ 2 / || J * Jt * (g - e) || ^ 2
	// Jt * (g - e) is a 9d vector, J * Jt * (g - e) is a 3d vector

	std::vector<float> top;
	glm::vec3 bot;

	//Jt * (g - e)
	for (int i = 0; i < 9; i++) {
		top.push_back(glm::dot(jacobian[i], goal - end_effector));
	}

	//J * Jt * (g - e)
	for (int i = 0; i < 3; i++) {

		float dot_product = 0;

		for (int j = 0; j < 9; j++) {
			dot_product += jacobian[j][i] * top[j];
		}

		bot[i] = dot_product;

	}

	//Get magnitude of top and bottom
	float top_magnitude = 0;
	float bot_magnitude = 0;

	for (int i = 0; i < 9; i++) {
		top_magnitude += powf(top[i], 2);
	}
	top_magnitude = powf(top_magnitude, 1.0f / 2);
	
	bot_magnitude = glm::length(bot);

	float step_size = (top_magnitude * top_magnitude) / (bot_magnitude * bot_magnitude);

	return step_size;

}

void Bone_Animation::change_dofs(glm::vec3 deltaE) {
	
	std::vector<float> delta_phi;

	for (int i = 0; i < 9; i++) {
		delta_phi.push_back(glm::dot(jacobian[i], deltaE));
	}

	for (int i = 1; i < rotation_degree_vector.size(); i++) {
		rotation_degree_vector[i].x += delta_phi[i * 3 - 3];
		rotation_degree_vector[i].y += delta_phi[i * 3 - 2];
		rotation_degree_vector[i].z += delta_phi[i * 3 - 1];

	}

	delta_phi.clear();

}

void Bone_Animation::traverse() {
	//Root bone
	Node* node_ptr = root_node;

	glm::mat4 mat = node_ptr->t_matrix;
	Transform_Data* transform = new Transform_Data(mat, node_ptr->data_ptr, glm::translate(mat, { 0.0f, 0.5f, 0.0f })[3]);
	bone_transforms.push_back(transform);

	//Traverse rest of tree;
	if (node_ptr->arc_ptr != NULL) {
		Arc* next_arc_ptr = node_ptr->arc_ptr;
		while (next_arc_ptr != NULL) {
			traversal_stack.push_back(mat);
			traverse(node_ptr->arc_ptr, mat);
			mat = traversal_stack.back();
			traversal_stack.pop_back();
			next_arc_ptr = next_arc_ptr->arc_ptr;
		}

	}

}

void Bone_Animation::traverse(Arc* arc_ptr, glm::mat4 mat) 
{
	mat = mat * arc_ptr->l_matrix;
	mat = mat * arc_ptr->a_matrix;
	
	Node* node_ptr = arc_ptr->node_ptr;

	mat = mat * node_ptr->t_matrix;
	Transform_Data* transform = new Transform_Data(mat, node_ptr->data_ptr, glm::translate(mat, {0.0f, 0.5f, 0.0f})[3]);
	bone_transforms.push_back(transform);

	if (node_ptr->arc_ptr != NULL) {
		Arc* next_arc_ptr = node_ptr->arc_ptr;
		while (next_arc_ptr != NULL) {
			traversal_stack.push_back(mat);
			traverse(next_arc_ptr, mat);
			mat = traversal_stack.back();
			traversal_stack.pop_back();
			next_arc_ptr = next_arc_ptr->arc_ptr;
		}

	}

}

// Creates the link tree, this is done every update in order to update the tree with the new transformations.
void Bone_Animation::create_tree() {

	//Create root
	glm::mat4 t_matrix = glm::mat4(1.0f);
	t_matrix = glm::translate(t_matrix, root_position);
	root_node = new Node(t_matrix, new Node_Data(colors[0], scale_vector[0], rotation_degree_vector[0]));

	Node* current_node = root_node;

	//Add the children
	for (int i = 1; i < scale_vector.size(); i++) {
		add_node(current_node, colors[i], scale_vector[i], rotation_degree_vector[i]);
		current_node = current_node->arc_ptr->node_ptr;

		Node_Data* data_ptr = current_node->data_ptr;

	}

}

void Bone_Animation::add_node(Node* parent, glm::vec4 color, glm::vec3 scale, glm::vec3 rotation) {

	//Create arc
	Node_Data* data_ptr = parent->data_ptr;


	// l_matrix translates bone to top of its parent
	glm::mat4 translation = glm::mat4(1.0f);
	translation[3] = glm::vec4(0.0f, 0.5f, 0.0f, 1.0f);
	glm::mat4 l_matrix = glm::mat4(1.0f);
	l_matrix = translation * l_matrix;
	
	// a_matrix does the inverse scale of the parent. I'm not sure what transformations are actually supposed to go the L, A, and T matrices, but the code functions like this so I did it this way.
	glm::mat4 a_matrix = glm::mat4(1.0f);
	a_matrix = glm::scale(a_matrix, 1.0f / data_ptr->scale);

	glm::mat4 t_matrix = create_t_matrix(scale, rotation);

	Node* node = new Node(t_matrix, new Node_Data(color, scale, rotation));
	
	parent->arc_ptr = new Arc(l_matrix, a_matrix, node);

}

glm::mat4 Bone_Animation::create_t_matrix(glm::vec3 scale, glm::vec3 rotation) {



	//Create t_matrix
	glm::mat4 t_matrix = glm::mat4(1.0f);
	//rotate y
	t_matrix = glm::rotate(glm::mat4(1.0f), glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f)) * t_matrix;
	//rotate z
	t_matrix = glm::rotate(glm::mat4(1.0f), glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f)) * t_matrix;
	//rotate x
	t_matrix = glm::rotate(glm::mat4(1.0f), glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f)) * t_matrix;
	//scale
	t_matrix = glm::scale(t_matrix, scale);
	//translation
	t_matrix = glm::translate(t_matrix, glm::vec3(0.0f, 0.5f, 0.0f));

	return t_matrix;
}

//Free up memory used by tree. Since I'm creating a new tree for each update, the last one should be deleted to free memory.
void Bone_Animation::delete_tree() {

	Node* node_ptr = root_node;
	Arc* arc_ptr = node_ptr->arc_ptr;
	
	if(arc_ptr != NULL)
		delete_tree(arc_ptr);

	delete(node_ptr);

	for (int i = 0; i < bone_transforms.size(); i++) {
		delete(bone_transforms[i]);
	}
	bone_transforms.clear();

}

void Bone_Animation::delete_tree(Arc* arc_ptr) {
	
	Node* node_ptr = arc_ptr->node_ptr;

	// Delete siblings
	if (arc_ptr->arc_ptr != NULL) 
		delete_tree(arc_ptr->arc_ptr);

	// Delete children
	if (node_ptr->arc_ptr != NULL)
		delete_tree(node_ptr->arc_ptr);
	
	delete(node_ptr);
	delete(arc_ptr);


}

void Bone_Animation::print_jacobian() {

	for (int i = 0; i < 9; i++) {
		std::cout << glm::to_string(jacobian[i]) << '\n';
	}

}

