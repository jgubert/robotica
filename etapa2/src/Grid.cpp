#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "Grid.h"
#include "math.h"

Grid::Grid ()
{
    mapScale_ = 10;
    mapWidth_ = mapHeight_ = 2000;
    numCellsInRow_=mapWidth_;
    halfNumCellsInRow_=mapWidth_/2;

    cells_ = new Cell[mapWidth_*mapHeight_];

    for (unsigned int j = 0; j < numCellsInRow_; ++j)
    {
        for (unsigned int i = 0; i < numCellsInRow_; ++i)
        {
            unsigned int c = j*numCellsInRow_ + i;
            cells_[c].x = -halfNumCellsInRow_ + 1 + i;
            cells_[c].y =  halfNumCellsInRow_ - j;

            cells_[c].occupancy = 0.5;
            cells_[c].logodds = 0;
            cells_[c].occupancySonar = 0.5;
            cells_[c].logoddsSonar = 0;
            cells_[c].himm = 7;
        }
    }

    numViewModes=4;
    viewMode=0;

    showValues=false;
}

Cell* Grid::getCell (int x, int y)
{
    int i=x+halfNumCellsInRow_-1;
    int j=halfNumCellsInRow_-y;
    return &(cells_[j*numCellsInRow_ + i]);
}

int Grid::getMapScale()
{
    return mapScale_;
}

int Grid::getMapWidth()
{
    return mapWidth_;
}

int Grid::getMapHeight()
{
    return mapHeight_;
}

void Grid::draw(int xi, int yi, int xf, int yf)
{
    glLoadIdentity();

    for(int i=xi; i<=xf; ++i){
        for(int j=yi; j<=yf; ++j){
            drawCell(i+j*numCellsInRow_);
        }
    }

    if(showValues){
        for(int i=xi; i<=xf; i++){
            for(int j=yi; j<=yf; j++){
                drawText(i+j*numCellsInRow_);
            }
        }
    }
}

void Grid::drawCell(unsigned int n)
{
    float aux;

    if(viewMode==0){
        aux=(1.0-cells_[n].occupancy);
    }else if(viewMode==1){
        aux=(1.0-cells_[n].occupancySonar);
    }else if(viewMode==2){
        aux=(16.0-cells_[n].himm)/16.0;
    }else{
        aux=1.0;
    }

    glColor3f(aux,aux,aux);

    glBegin( GL_QUADS );
    {
        glVertex2f(cells_[n].x+1, cells_[n].y+1);
        glVertex2f(cells_[n].x+1, cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y+1);
    }
    glEnd();
}

void Grid::drawText(unsigned int n)
{
    glRasterPos2f(cells_[n].x+0.25, cells_[n].y+0.25);
    std::stringstream s;
    glColor3f(0.5f, 0.0f, 0.0f);
    s << cells_[n].occupancy;

    std::string text=s.str();
    for (unsigned int i=0; i<text.size(); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
    }
}

void Cell::updateOccupancyFromLogOdds()
{
    occupancy = 1.0 - 1.0/(1.0+exp(logodds));
}
