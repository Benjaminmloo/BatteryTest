#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "TimeSeriesGraph.h"

//TODO: if time exceeds present max add 1 minute to bottom axis
TimeSeriesGraph::TimeSeriesGraph(
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
    _graphIndex = 0;

  }

void TimeSeriesGraph::drawGrid() {
  int i;
  float temp;

  //value between grid lines
  float incSizeT = _domain / _numIncT;
  float incSizeY = _range / _numIncY;

  CLEAR_GRAPH_AREA;

  _d.fillRect(0, 0, _GRAPH_X + _GRAPH_W, _GRAPH_Y, _BACK_COLOUR);

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

void TimeSeriesGraph::drawValue(float x, float y) {
  if(_graphIndex >= _GRAPH_W){
    extendDomain(_domain  + 30);
  }
  if(floatToGraphPos(x, _minT, _domain, _GRAPH_W, _GRAPH_X) >= _graphIndex + _GRAPH_X) {
    _graphStore[_graphIndex] = floatToGraphPos(-y, _minY, _range, _GRAPH_H, _GRAPH_Y);

    _d.drawPixel(_graphIndex + _GRAPH_X, _graphStore[_graphIndex++], _POINT_COLOUR);
  }
}

void TimeSeriesGraph::extendDomain (float newDomain){
  float newT = newDomain / _GRAPH_W;
  float oldT = _domain / _GRAPH_W;
  int oldIndex;

  _graphIndex = 0;

  for(oldIndex = 0; oldIndex < _GRAPH_W; oldIndex++){
    while(oldIndex * oldT >= _graphIndex * newT)
      _graphIndex ++;
    _graphStore[_graphIndex] = _graphStore[oldIndex];
  }

  _graphIndex = (_domain / newDomain) * (float) _GRAPH_W;

  _domain = newDomain;
  drawGrid();
  for(oldIndex = 0; oldIndex < _graphIndex; oldIndex++){
    _d.drawPixel(oldIndex + _GRAPH_X, _graphStore[oldIndex], _POINT_COLOUR);
  }
}

void TimeSeriesGraph::reset(){
  _graphIndex = 0;
  drawGrid();
}

int TimeSeriesGraph::floatToGraphPos(float n, float min, float range, int graphRange, int graphOffset){
  return (n - min) * graphRange / (range) + graphOffset;
}
