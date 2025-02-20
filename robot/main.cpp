//SFML .dlls are needed for this to work
#include <SFML/Graphics.hpp>
#include <iostream>
#include <functional>
#include <unordered_set>
#include <set>
#include <unordered_map>

using namespace std;
using namespace sf;
const int mapQuadsWide = 10;
const int mapQuadsHigh = 10;
const int quadSize = 50;
const int robotSixe = 45;

const int movementCost = 5;

int numSearchNodes = 0;//number of search nodes generated when looking for a solution

const Vector2i mapOffset(70, 0);

//sets the colour of a single quad, quadCoord indexed start at zero
void setQuadColour(VertexArray& map, Vector2i quadCoord, Color colour)
{
	int quadIndex = ((quadCoord.y * mapQuadsWide) + quadCoord.x) * 4;
	map[quadIndex].color = colour;
	map[quadIndex + 1].color = colour;
	map[quadIndex + 2].color = colour;
	map[quadIndex + 3].color = colour;
}
//map cood positions are set from the top left going from left to right in rows downward
//sets the vertex positions in the vertex array that will represent the map to the user
void setUpMap(VertexArray& map)
{
	Vector2i bottomLeftCoord = Vector2i(mapOffset.x + mapQuadsWide * quadSize, mapOffset.y + mapQuadsHigh * quadSize);
	int vertexIndex = 0;
	for (int y = mapOffset.y; y < bottomLeftCoord.y; y += quadSize)
	{
		for (int x = mapOffset.x; x < bottomLeftCoord.x; x += quadSize)
		{
			//the x + 1 etc are a simple quick way to add gridlines
			map[vertexIndex].color = Color::Green;
			map[vertexIndex++].position = Vector2f(x + 1, y + 1);
			map[vertexIndex].color = Color::Green;
			map[vertexIndex++].position = Vector2f(x + quadSize - 1 , y + 1);
			map[vertexIndex].color = Color::Green;
			map[vertexIndex++].position = Vector2f(x + quadSize - 1 , y + quadSize - 1);
			map[vertexIndex].color = Color::Green;
			map[vertexIndex++].position = Vector2f(x + 1, y + quadSize - 1);
		}
	}
}
//G - cost(g) → "G" for Ground cost(actual cost traveled so far).
//H - cost(h) → "H" for Heuristic cost(estimated cost to the goal). ....THESE THREE LINES ARE FROM CHAT GPT
//F - cost(f) → "F" for Final cost(f = g + h, total estimated cost).
class RobotState
{
public:
	Vector2i position;

	// Define equality operator (required for unordered_set)
	bool operator==(const RobotState& other) const {
		if (position.x != other.position.x || position.y != other.position.y)
		{
			return false;
		}
		return true;
	}
};

class Action
{
protected:

public:
	Action() {

	}
	float cost;
	virtual bool isPossible(RobotState& currentPosition) = 0;
	virtual void apply(RobotState& state) = 0;
	~Action() = default;
};

enum movementType
{
	UP,
	DOWN,
	LEFT,
	RIGHT
};

class MovementAction : public Action
{
private:
	int deltaX;
	int deltaY;
public:
	MovementAction(int dx, int dy, float _cost)
		: deltaX(dx), deltaY(dy)
	{
		cost = _cost;
	}

	void apply(RobotState& state) override
	{
		state.position.x += deltaX;
		state.position.y += deltaY;
	}

	bool isPossible(RobotState& state) override
	{
		int newXCoord = state.position.x + deltaX;
		int newYCoord = state.position.y + deltaY;
		if (newXCoord >= 0 && newYCoord >= 0)
		{
			if (newXCoord < mapQuadsWide && newYCoord < mapQuadsHigh)
			{
				return true;
			}
		}
		return false;
	}
};

Action* movementActionFactory(movementType type, float cost = movementCost)
{
	//(0, 0) coord is the top left of window with positive indexes moving down and right
	switch (type)
	{
	case UP:
		return new MovementAction(0, -1, cost);
	case DOWN:
		return new MovementAction(0, 1, cost);
	case LEFT:
		return new MovementAction(-1, 0, cost);
	case RIGHT:
		return new MovementAction(1, 0, cost);
	default:
		break;
	}
}



class Robot
{

public:
	RobotState state;
	RobotState goalState;
	vector<Action*> actions;
	vector<int> plan;
	Vector2f shownPosition;
	VertexArray square;

	Robot(Vector2i initialPosition, VertexArray& _map)
	{
		state.position = initialPosition;
		shownPosition = Vector2f(initialPosition.x, initialPosition.y);
		square = VertexArray(Quads, 4);
		actions.push_back(movementActionFactory(movementType::DOWN));
		actions.push_back(movementActionFactory(movementType::LEFT));
		actions.push_back(movementActionFactory(movementType::RIGHT));
		actions.push_back(movementActionFactory(movementType::UP));
		draw();
	}

	void run()
	{
		if (plan.size() > 0)
		{
			Action* nextAction = actions[plan[0]];
			nextAction->apply(this->state);
			shownPosition = (Vector2f)this->state.position;
			plan.erase(plan.begin());
			draw();
		}
	}

	void setGoalState(RobotState state)
	{
		goalState = state;
	}

	//set color in map vertex array to show robot
	void draw()
	{
		float screenPositionX = shownPosition.x * quadSize;

		float screenPositionY = shownPosition.y * quadSize;

		float off = (quadSize - robotSixe) / 2;
		square[0].color = Color::Blue;
		square[0].position = Vector2f(screenPositionX + off, screenPositionY + off);
		square[1].color = Color::Blue;
		square[1].position = Vector2f(screenPositionX + robotSixe + off, screenPositionY + off);
		square[2].color = Color::Blue;
		square[2].position = Vector2f(screenPositionX + robotSixe + off, screenPositionY + robotSixe + off);
		square[3].color = Color::Blue;
		square[3].position = Vector2f(screenPositionX + off, screenPositionY + robotSixe + off);

		square[0].position.x += mapOffset.x;
		square[0].position.y += mapOffset.y;
		square[1].position.x += mapOffset.x;
		square[1].position.y += mapOffset.y;
		square[2].position.x += mapOffset.x;
		square[2].position.y += mapOffset.y;
		square[3].position.x += mapOffset.x;
		square[4].position.y += mapOffset.y;
	}
};

class Action;
class SearchNode
{
	
public:
	SearchNode(float gCost, float tCost, RobotState& _state, SearchNode* _parentNode, int index)
	{
		numSearchNodes++;
		groundCost = gCost;
		totalCost = tCost;
		parentNode = _parentNode;
		actionIndex = index;
		state = _state;
	}
	float groundCost;//cost from start to this node
	float totalCost;//g cost + h cost
	int actionIndex;//index of the where the action taken to reach this state is located in the robot's action vector
	SearchNode* parentNode;
	RobotState state;
};

struct RobotStateHash {
	size_t operator()(const RobotState& state) const {
		return hash<int>()(state.position.x) ^ (hash<int>()(state.position.y) << 1);
	}
};

class AStarPlanner
{
private:
	vector<SearchNode*> openSearchNodes;
	unordered_set<RobotState, RobotStateHash> searchedStates;
	function<float(RobotState, RobotState)> heuristic;
public:
	AStarPlanner(function<float(RobotState, RobotState)> initialHeuristic)
	{
		openSearchNodes = vector<SearchNode*>();
		searchedStates = unordered_set<RobotState, RobotStateHash>();
		heuristic = initialHeuristic;
	}
	void setHeuristic(function<float(RobotState, RobotState)> newHeuristic)
	{
		heuristic = newHeuristic;
	}
	void addOpenNode(float gCost, float tCost, RobotState& _state, SearchNode* _parentNode, int index)
	{
		SearchNode* newNode = new SearchNode(gCost, tCost, _state, _parentNode, index);
		openSearchNodes.push_back(newNode);
	}
	void closeTopOpenNode(SearchNode* nodeToDelete)
	{
		for (auto node = openSearchNodes.begin(); node < openSearchNodes.end(); node++)
		{
			if ((*node)->state == nodeToDelete->state)
			{
				
				searchedStates.insert((*node)->state);
				openSearchNodes.erase(node);
				break;
			}
		}
	}
	SearchNode* getCheapestOpenNode()
	{
		float cheapestCost = INT32_MAX;
		SearchNode* cheapestNode = nullptr;

		for (SearchNode* node : openSearchNodes)
		{
			if (node->totalCost < cheapestCost)
			{
				cheapestCost = node->totalCost;
				cheapestNode = node;
			}
		}

		return cheapestNode;
	}
	vector<int> generatePlan(Robot& robot)
	{
		numSearchNodes = 0;
		vector<Action*> actions = robot.actions;
		addOpenNode(0, 0, robot.state, nullptr, -1);
		

		while (openSearchNodes.empty() != true)
		{
			SearchNode* currentStateNode = getCheapestOpenNode();
			RobotState currentState = currentStateNode->state;

			float currentGCost = currentStateNode->groundCost;
			int actionIndex = -1;//index of the action within the robot's actions vector
			for (Action* action : actions)
			{
				actionIndex++;
				//compute new state for each potential action
				if (action->isPossible(currentState))
				{
					//calculate potential new state
					RobotState potentialNewState(currentState);
					action->apply(potentialNewState);

					//check if the potential new state is the goal state
					if (potentialNewState == robot.goalState)
					{
						//build a vector of actionIndexes
						vector<int> actions = vector<int>();
						actions.push_back(actionIndex);
						SearchNode* node = currentStateNode;
						while (node->parentNode != NULL)
						{
							actions.push_back(node->actionIndex);
							node = node->parentNode;
						}
						cout << "plan generated using " << numSearchNodes << " search nodes" << endl;
						return actions;
					}

					//discard new state if it is contained in the closed list 
					if (searchedStates.find(potentialNewState) != searchedStates.end())
					{
						continue;
					}
					//compute the g and h costs
					float newGCost = currentGCost + action->cost;

					float newHCost = heuristic(potentialNewState, robot.goalState);

					float totalCost = newGCost + newHCost;
					//discard new state if it is in the open list with a lower g cost
					for (auto node = openSearchNodes.begin(); node < openSearchNodes.end(); node++)
					{
						if ((*node)->state == potentialNewState)
						{
							if ((*node)->groundCost <= newGCost)
							{
								openSearchNodes.erase(node);//delete old more expensive state
								break;

							}
							else
							{
								continue;//discard new state
							}
						}
					}
					//place the new state reached here onto the open list
					addOpenNode(newGCost, totalCost, potentialNewState, currentStateNode, actionIndex);
					
				}
			}
			//add the currentStateNode state, which has had all possible actions searched, to the closed list
			closeTopOpenNode(currentStateNode);

		}
		//return no plan
	}
};

//AN ALTERNATIVE IS HAVING HEURISTIC CLASSES WHICH CAN BE SET WITH, AND ONLY USE, THE DATA THEY ACTUALLY NEED

float manhattanHeuristic(RobotState startState, RobotState goalState)
{
	Vector2i startPosition = startState.position;
	Vector2i goalPosition = goalState.position;
	return (abs(goalPosition.x - startPosition.x) + abs(goalPosition.y - startPosition.y)) * movementCost * 1.5;
}

float euclideanHeursitic(RobotState startState, RobotState goalState)
{
	Vector2i startPosition = startState.position;
	Vector2i goalPosition = goalState.position;
	return std::sqrt(std::pow(goalPosition.x - startPosition.x, 2) + std::pow(goalPosition.y - startPosition.y, 2)) * movementCost * 1.5;
	
}


int main()
{
	RenderWindow window(VideoMode(600, 600), "Mandelbrot");
	VertexArray map = VertexArray(Quads, (mapQuadsWide * mapQuadsHigh) * 4);
	setUpMap(map);

	Robot robot = Robot(Vector2i(0, 0), map);

	RobotState goalState = RobotState();
	goalState.position.x = 9;
	goalState.position.y = 9;								//USE PYTHAG HEIURISTIC

	robot.setGoalState(goalState);

	AStarPlanner planner = AStarPlanner(manhattanHeuristic);
//	AStarPlanner planner = AStarPlanner(euclideanHeursitic);
	vector<int> actions = planner.generatePlan(robot);
	cout << "Number of actions: " << actions.size() << endl;
	for (int i : actions)
	{
		//copy actions into robot's plan
		cout << i << endl;
		robot.plan.push_back(i);
	}
	

	const int frameRateLimit = 1;
	window.setFramerateLimit(5);

	while (window.isOpen())
	{
		Event event;
		while (window.pollEvent(event))
		{
			if (event.type == Event::Closed)
			{
				window.close();
			}
		}

		robot.run();


		window.clear();
		window.draw(map);
		window.draw(robot.square);
		window.display();
	}
	return 0;
}