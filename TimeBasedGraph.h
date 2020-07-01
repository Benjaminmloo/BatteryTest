#ifndef TimeBasedGraph_h
#define TimeBasedGraph_h

#include "Arduino.h"

//template used bc it initialises the size of the
//allowing for screens of diferent sizes
class TimeBasedGraph{
  public:
    TimeBasedGraph(
      Adafruit_ILI9341 &d,
      float maxT, int numIncT,
      float minY, float range, int numIncY, String ylabel,
      int graphX, int graphY, int graphW, int graphH,
      uint16_t gridColour, uint16_t axisColour,
      uint16_t pointColour, uint16_t backColour,
      uint16_t textColour);

    void drawGraphGrid();
    void drawGraphValue(float x, float y);

  private:
    Adafruit_ILI9341 &_d;
    float _minT; //lowest value on scale
    float _domain; //expected range from min to max values
    int _numIncT; //number of increments to draw on the axis
    float _minY;
    float _range;
    int _numIncY;
    String _ylabel;

    //Poistion of graph on the display
    const int _GRAPH_X;
    const int _GRAPH_Y;

    //dimensions of the graph in pixels
    const int _GRAPH_W;
    const int _GRAPH_H;

    const uint16_t  _GRID_COLOUR;
    const uint16_t  _AXIS_COLOUR;
    const uint16_t  _POINT_COLOUR;
    const uint16_t  _BACK_COLOUR;
    const uint16_t  _TEXT_COLOUR;

    int *_graphStore; //stores values previously printed to the screen
    int _graphIndex;


    int floatToGraphPos(float n, float min, float range, int graphRange, int graphOffset);
};

#endif
