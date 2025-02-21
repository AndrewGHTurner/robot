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
const int robotSize = 45;
const int cakeSize = 20;

const int movementCost = 5;
const int cakeMovementCost = 3;
const int pickUpPutDownCost = 10;

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
	bool holdingCake = false;

	// Define equality operator (required for unordered_set)
	bool operator==(const RobotState& other) const {

		if (holdingCake != other.holdingCake)
		{
			return false;
		}

		if (position.x != other.position.x || position.y != other.position.y)
		{
			return false;
		}
		return true;
	}
};

class WorldState
{
public:
	Vector2i cakePosition;

	WorldState()
	{

	}

	bool operator==(const WorldState& other) const {
		if (cakePosition.x != other.cakePosition.x || cakePosition.y != other.cakePosition.y)
		{
			return false;
		}
		return true;
	}
};

class RobotMentalState
{
public:
	RobotState robot;
	WorldState world;
	RobotMentalState() {}
	RobotMentalState(RobotState rb, WorldState ws)
	{
		robot = rb;
		world = ws;
	}

	bool operator==(const RobotMentalState& other) const {
		if (!(robot == other.robot))
		{
			return false;
		}
		if (!(world == other.world))
		{
			return false;
		}
		return true;
	}
};

WorldState ActualWorldState = WorldState();

class Action
{
protected:

public:
	Action() {

	}
	float cost;
	virtual bool isPossible(RobotMentalState& mentalState) = 0;
	virtual void apply(RobotState& robot, WorldState& world) = 0;
	~Action() = default;
};

class PutDownAction : public Action
{
public:
	PutDownAction(float _cost)
	{
		cost = _cost;
	}
	void apply(RobotState& robot, WorldState& world) override
	{
		robot.holdingCake = false;
	}
	bool isPossible(RobotMentalState& mentalState) override
	{
		return mentalState.robot.holdingCake;
	}
};

class PickUpAction : public Action
{
public:
	PickUpAction(float _cost)
	{
		cost = _cost;
	}
	void apply(RobotState& robot, WorldState& world) override
	{
		robot.holdingCake = true;
	}
	bool isPossible(RobotMentalState& mentalState) override
	{
		if (mentalState.robot.holdingCake)
		{
			return false;
		}
		if (mentalState.robot.position.x == mentalState.world.cakePosition.x)
		{
			if (mentalState.robot.position.y == mentalState.world.cakePosition.y)
			{
				return true;
			}
		}
		return false;
	}
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

	void apply(RobotState& robot, WorldState& world) override
	{
		robot.position.x += deltaX;
		robot.position.y += deltaY;
		if (robot.holdingCake)
		{
			world.cakePosition.x += deltaX;
			world.cakePosition.y += deltaY;
		}
	}

	bool isPossible(RobotMentalState& mentalState) override
	{
		int newXCoord = mentalState.robot.position.x + deltaX;
		int newYCoord = mentalState.robot.position.y + deltaY;
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
	RobotMentalState goalState;
	RobotMentalState knownWorldState;
	vector<Action*> actions;
	vector<int> plan;
	Vector2f shownPosition;
	VertexArray square;

	Robot(Vector2i initialPosition, VertexArray& _map)
	{
		knownWorldState = RobotMentalState(RobotState(), WorldState());
		knownWorldState.robot.position = initialPosition;
		knownWorldState.robot.holdingCake = false;
		goalState = RobotMentalState(RobotState(), WorldState());
		shownPosition = Vector2f(initialPosition.x, initialPosition.y);
		square = VertexArray(Quads, 4);
		actions.push_back(movementActionFactory(movementType::DOWN));
		actions.push_back(movementActionFactory(movementType::LEFT));
		actions.push_back(movementActionFactory(movementType::RIGHT));
		actions.push_back(movementActionFactory(movementType::UP));
		actions.push_back(new PickUpAction(pickUpPutDownCost));
		actions.push_back(new PutDownAction(pickUpPutDownCost));
		draw();
	}

	void run()
	{
		if (plan.size() > 0)
		{
			Action* nextAction = actions[plan[0]];
			nextAction->apply(this->knownWorldState.robot, ActualWorldState);

			//update the robots known world state
			knownWorldState.world = ActualWorldState;

			shownPosition = (Vector2f)this->knownWorldState.robot.position;
			plan.erase(plan.begin());
			draw();
		}
	}

	void setGoalState(RobotMentalState state)
	{
		goalState = state;
	}

	//set color in map vertex array to show robot
	void draw()
	{
		float screenPositionX = shownPosition.x * quadSize;

		float screenPositionY = shownPosition.y * quadSize;

		float off = (quadSize - robotSize) / 2;
		square[0].color = Color::Blue;
		square[0].position = Vector2f(screenPositionX + off, screenPositionY + off);
		square[1].color = Color::Blue;
		square[1].position = Vector2f(screenPositionX + robotSize + off, screenPositionY + off);
		square[2].color = Color::Blue;
		square[2].position = Vector2f(screenPositionX + robotSize + off, screenPositionY + robotSize + off);
		square[3].color = Color::Blue;
		square[3].position = Vector2f(screenPositionX + off, screenPositionY + robotSize + off);

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
	SearchNode(float gCost, float tCost, RobotMentalState& _state, SearchNode* _parentNode, int index)
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
	RobotMentalState state;
};

struct RobotMentalStateHash {
	size_t operator()(const RobotMentalState& mentalState) const {
		return hash<int>()(mentalState.robot.position.x) ^ (hash<int>()(mentalState.robot.position.y) << 1);
	}
};

class AStarPlanner
{
private:
	vector<SearchNode*> openSearchNodes;
	unordered_set<RobotMentalState, RobotMentalStateHash> searchedStates;
	function<float(RobotMentalState, RobotMentalState)> heuristic;
public:
	AStarPlanner(function<float(RobotMentalState, RobotMentalState)> initialHeuristic)
	{
		openSearchNodes = vector<SearchNode*>();
		searchedStates = unordered_set<RobotMentalState, RobotMentalStateHash>();
		heuristic = initialHeuristic;
	}
	void setHeuristic(function<float(RobotMentalState, RobotMentalState)> newHeuristic)
	{
		heuristic = newHeuristic;
	}
	void addOpenNode(float gCost, float tCost, RobotMentalState& _state, SearchNode* _parentNode, int index)
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
		searchedStates.clear();
		openSearchNodes.clear();
		numSearchNodes = 0;
		vector<Action*> actions = robot.actions;
		addOpenNode(0, 0, robot.knownWorldState, nullptr, -1);
		

		while (openSearchNodes.empty() != true)
		{
			SearchNode* currentStateNode = getCheapestOpenNode();
			RobotMentalState currentState = currentStateNode->state;

			float currentGCost = currentStateNode->groundCost;
			int actionIndex = -1;//index of the action within the robot's actions vector
			for (Action* action : actions)
			{
				actionIndex++;
				//compute new state for each potential action
									//calculate potential new state
				RobotMentalState potentialNewState(currentState);
				if (action->isPossible(currentState))
				{

					action->apply(potentialNewState.robot, potentialNewState.world);

					//check if the potential new state is the goal state
					if (potentialNewState == robot.goalState)
					{
						//build a vector of actionIndexes
						vector<int> actions = vector<int>();
						actions.push_back(actionIndex);
						SearchNode* node = currentStateNode;
						while (node->parentNode != NULL)
						{
							actions.insert(actions.begin(), node->actionIndex);
							node = node->parentNode;
						}
						cout << "used " << numSearchNodes << " search nodes" << endl;
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
		cout << "No possible plan" << endl;
		//return no plan
	}
};

//AN ALTERNATIVE IS HAVING HEURISTIC CLASSES WHICH CAN BE SET WITH, AND ONLY USE, THE DATA THEY ACTUALLY NEED

int manhattanDistance(Vector2i startPosition, Vector2i endPosition)
{
	return abs(startPosition.x - endPosition.x) + abs(startPosition.y - endPosition.y);
}

float manhattanHeuristic(RobotMentalState startState, RobotMentalState goalState)
{
	float finalCost = 0;
	//if the cake is not at the goal location
	if (goalState.world.cakePosition != startState.world.cakePosition)
	{
		//if the robot is not holding the cake
		if (!startState.robot.holdingCake)
		{
			//add the cost of picking up the cake
			finalCost += pickUpPutDownCost;
		}
		//if the robot is not at the location of the cake
		if (startState.robot.position != startState.world.cakePosition)
		{
			//add the cost of moving towards the cake
			finalCost += (manhattanDistance(startState.robot.position, startState.world.cakePosition) * movementCost);
		}
		//if the goal state is to not be holding the cake
		if (goalState.robot.holdingCake == false)
		{
			//add the cost of putting down the cake
			finalCost += pickUpPutDownCost;
		}
	}
	//add cost of moving the cake
	finalCost += (manhattanDistance(startState.world.cakePosition, goalState.world.cakePosition) * cakeMovementCost);
	//add the cost of moving the robot to the goal position
	finalCost += (manhattanDistance(startState.robot.position, goalState.robot.position) * movementCost);
	return finalCost * 1.5;
}

float euclideanHeursitic(RobotMentalState startState, RobotMentalState goalState)
{
	Vector2i startPosition = startState.robot.position;
	Vector2i goalPosition = goalState.robot.position;
	return std::sqrt(std::pow(goalPosition.x - startPosition.x, 2) + std::pow(goalPosition.y - startPosition.y, 2)) * movementCost * 1.5;
}


int main()
{
	RenderWindow window(VideoMode(600, 600), "Mandelbrot");
	VertexArray map = VertexArray(Quads, (mapQuadsWide * mapQuadsHigh) * 4);
	setUpMap(map);

	sf::CircleShape circle(cakeSize);
	circle.setFillColor(sf::Color::Red);


	Robot robot = Robot(Vector2i(0, 0), map);

	//set the goal state
	RobotState goalRobotState = RobotState();
	goalRobotState.position.x = 2;
	goalRobotState.position.y = 9;
	goalRobotState.holdingCake = false;
	WorldState goalWorldState = WorldState();
	goalWorldState.cakePosition.x = 3;
	goalWorldState.cakePosition.y = 8;
	RobotMentalState goalMentalState = RobotMentalState(goalRobotState, goalWorldState);
	//set the actual world state
	ActualWorldState.cakePosition.x = 8;
	ActualWorldState.cakePosition.y = 2;

	robot.knownWorldState.world = ActualWorldState;

	robot.setGoalState(goalMentalState);

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

	bool cakeSelected = false;
	Vector2i selectedQuad = Vector2i(-1, -1);
	sf::RectangleShape selectionIndicator(sf::Vector2f(quadSize, quadSize));
	selectionIndicator.setFillColor(sf::Color::Transparent); 
	selectionIndicator.setOutlineThickness(5.f);  
	selectionIndicator.setOutlineColor(sf::Color::Yellow);

	string state = "nothing selected";

	while (window.isOpen())
	{
		Event event;
		while (window.pollEvent(event))
		{
			if (event.type == Event::Closed)
			{
				window.close();
			}
			if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
			{
				cout << state << endl;
				sf::Vector2i mousePosition = sf::Mouse::getPosition(window);
				Vector2i mouseMapPosition = Vector2i(mousePosition.x - mapOffset.x, mousePosition.y - mapOffset.y);
				cout << mouseMapPosition.x << ", " << mouseMapPosition.y << endl;
				Vector2i quadCoord = Vector2i(mouseMapPosition.x / quadSize, mouseMapPosition.y / quadSize);
				cout << quadCoord.x << ", " << quadCoord.y << endl;

				if (state == "selecting new robot position")
				{
					robot.goalState.robot.position = quadCoord;
					state = "nothing selected";
				}
				else if (state == "selecting new cake position")
				{
					robot.goalState.world.cakePosition = quadCoord;
					state = "nothing selected";
				}
				else if (quadCoord == ActualWorldState.cakePosition)
				{
					state = "selecting new cake position";
					Vector2f indicatorPosition = Vector2f((quadCoord.x * quadSize) + mapOffset.x, (quadCoord.y * quadSize) + mapOffset.y);
					selectionIndicator.setPosition(indicatorPosition);
				}
				else if (quadCoord == robot.knownWorldState.robot.position)
				{
					state = "selecting new robot position";
					Vector2f indicatorPosition = Vector2f((quadCoord.x * quadSize) + mapOffset.x, (quadCoord.y * quadSize) + mapOffset.y);
					selectionIndicator.setPosition(indicatorPosition);
				}
			}
		}

		
		float off = (quadSize - cakeSize);
		float circleXCoord = (ActualWorldState.cakePosition.x * quadSize - (0.5 * quadSize)) + mapOffset.x + off;
		float circleYCoord = (ActualWorldState.cakePosition.y * quadSize - (0.5 * quadSize)) + mapOffset.y + off;
		circle.setPosition(Vector2f(circleXCoord, circleYCoord));

		window.clear();
		window.draw(map);
		window.draw(robot.square);
		window.draw(circle);
		if (state != "nothing selected")
		{
			window.draw(selectionIndicator);
		}
		
		robot.run();

		//SHOULD MOVE THIS INTO THE ROBOT RUN METHOD
		if (robot.plan.size() == 0)
		{
			if (!(robot.knownWorldState == robot.goalState))
			{
				vector<int> actions = planner.generatePlan(robot);
				cout << "Number of actions: " << actions.size() << endl;
				for (int i : actions)
				{
					//copy actions into robot's plan
					cout << i << endl;
					robot.plan.push_back(i);
				}
			}
		}


		window.display();
	}
	return 0;
}

//add pick up and put down actions
//create new heurostic function
