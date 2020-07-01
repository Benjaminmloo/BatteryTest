#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "TimeBasedGraph.h"

//TODO: if time exceeds present max add 1 minute to bottom axis
TimeBasedGraph::TimeBasedGraph(
  Adafruit_ILI9341 &d,
  float maxT, int numIncT,
  float minY, float range, int numIncY, String ylabel,
  int graphX, int graphY, int graphW, int graphH,
  uint16_t gridColour, uint16_t axisColour,
  uint16_t pointColour, uint16_t backColour,
  uint16_t textColour):
  _d(d),
  _minT(0), _domain(maxT), _numIncT(numIncT),
  _minY(minY), _range(range), _numIncY(numIncY), _ylabel(ylabel),
  _GRAPH_X(graphX), _GRAPH_Y(graphY), _GRAPH_W(graphW), _GRAPH_H(graphH),
  _GRID_COLOUR(gridColour), _AXIS_COLOUR(axisColour),
  _POINT_COLOUR(pointColour), _BACK_COLOUR(backColour),
  _TEXT_COLOUR(textColour){
    _graphStore = new int[_GRAPH_W];

  }

void TimeBasedGraph::drawGraphGrid() {
  int i;
  float temp;

  //value between grid lines
  float incSizeT = _domain / _numIncT;
  float incSizeY = _range / _numIncY;

  _graphIndex = 0;

  //TODO: draw a black rectangle around everything
  _d.setTextSize(1);
  _d.setTextColor(_TEXT_COLOUR, _BACK_COLOUR);

//draw axis lines w/ zero  label
  temp =  floatToGraphPos(0, _minY, _range, _GRAPH_H, _GRAPH_Y);
  _d.drawLine(_GRAPH_X, temp, _GRAPH_X + _GRAPH_W, temp, _AXIS_COLOUR);
  _d.setCursor(_GRAPH_X - 30, temp);
  _d.println(0, 1);

  temp =  floatToGraphPos(0, _minT, _domain, _GRAPH_W, _GRAPH_X);
  _d.drawLine(temp, _GRAPH_Y, temp, _GRAPH_Y - _GRAPH_H, _AXIS_COLOUR);
  _d.setCursor(temp, _GRAPH_Y + 10);
  _d.println(0, 1);

  // draw the scale lines w/ numerical labels
  for (i = 1; i <= _numIncY; i ++) {
    temp =  floatToGraphPos(i * -incSizeY, _minY, _range, _GRAPH_H, _GRAPH_Y);
    _d.drawLine(_GRAPH_X, temp, _GRAPH_X + _GRAPH_W, temp, _GRID_COLOUR);

    _d.setCursor(_GRAPH_X - 30, temp);
    _d.println(i * incSizeY, 1);
  }

  for (i = 1; i <= _numIncT; i ++) {
    temp =  floatToGraphPos(i * incSizeT, _minT, _domain, _GRAPH_W, _GRAPH_X);
    _d.drawLine(temp, _GRAPH_Y, temp, _GRAPH_Y - _GRAPH_H, _GRID_COLOUR);

    _d.setCursor(temp, _GRAPH_Y + 10);
    _d.println(i * incSizeT, 1);
  }

  //now draw the axis labels

  _d.setTextSize(1);
  _d.setTextColor(_AXIS_COLOUR, _BACK_COLOUR);

  _d.setCursor(_GRAPH_X - 10, _GRAPH_Y + 10);
  _d.println('t');

  _d.setCursor(_GRAPH_X - 30, 0);
  _d.println(_ylabel);
}

void TimeBasedGraph::drawGraphValue(float x, float y) {
  if(floatToGraphPos(x, _minT, _domain, _GRAPH_W, _GRAPH_X) >= _graphIndex + _GRAPH_X) {
    _graphStore[_graphIndex] = floatToGraphPos(-y, _minY, _range, _GRAPH_H, _GRAPH_Y);

    _d.drawPixel(_graphIndex + _GRAPH_X, _graphStore[_graphIndex++], _POINT_COLOUR);
  }
}


int TimeBasedGraph::floatToGraphPos(float n, float min, float range, int graphRange, int graphOffset){
  return (n - min) * graphRange / (range) + graphOffset;
}
