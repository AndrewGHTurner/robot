//SFML .dlls are needed for this to work
#include <SFML/Graphics.hpp>
#include <iostream>
#include <queue>
using namespace std;
using namespace sf;
const int mapQuadsWide = 10;
const int mapQuadsHigh = 10;
const int quadSize = 50;

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
void setUpMap(VertexArray& map, Vector2i topLeftCoord)
{
	Vector2i bottomLeftCoord = Vector2i(topLeftCoord.x + mapQuadsWide * quadSize, topLeftCoord.y + mapQuadsHigh * quadSize);
	int vertexIndex = 0;
	for (int y = topLeftCoord.y; y < bottomLeftCoord.y; y += quadSize) 
	{
		for (int x = topLeftCoord.x; x < bottomLeftCoord.x; x += quadSize)
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

class ActionNode
{
	
public:
	float cost;
};

struct Compare {
	bool operator()(const ActionNode& node1, const ActionNode& node2) const {
		return node1.cost > node2.cost;  // Min-heap (lowest cost has highest priority)
	}
};

class AStarPlanner
{
private:
	priority_queue<ActionNode, vector<ActionNode>, Compare> openList;
	priority_queue<ActionNode, vector<ActionNode>, Compare> closedList;
	
public:
	AStarPlanner()
	{
		openList = priority_queue<ActionNode, vector<ActionNode>, Compare>();
		closedList = priority_queue<ActionNode, vector<ActionNode>, Compare>();
	}
	void setStart()
};

class Robot
{
	Vector2i position;
	VertexArray& map;
	Vector2i goalPosition = Vector2i(2, 2);
public:
	

	Robot(Vector2i initialPosition, VertexArray& _map) : map(_map)
	{
		position = initialPosition;
		draw();
	}

	//set color in map vertex array to show robot
	void draw()
	{
		setQuadColour(map, position, Color::Blue);
	}
};



int main()
{
	RenderWindow window(VideoMode(600, 600), "Mandelbrot");
	VertexArray map = VertexArray(Quads, (mapQuadsWide * mapQuadsHigh) * 4);
	setUpMap(map, Vector2i(70, 0));

	Robot(Vector2i(5, 6), map);

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
		window.clear();
		window.draw(map);
		window.display();
	}
	return 0;
}