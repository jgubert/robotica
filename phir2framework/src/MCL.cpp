#include "MCL.h"

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#define PI 3.1415

#include <GL/glut.h>

MCL::MCL(float maxRange, std::string mapName, pthread_mutex_t* m):
    maxRange(maxRange), mutex(m)
{
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);

    readMap(mapName);
    scale = 10;
    transparency = false;

    numParticles = 10000;

    initParticles();
}

void MCL::run(const Action &u, const std::vector<float> &z)
{
    sampling(u);
    weighting(z);
    resampling();

    updateMeanAndCovariance();
}


//////////////////////////////////////////////////
//// Métodos SAMPLING, WEIGHTING e RESAMPLING ////
//////////////////////////////////////////////////

void MCL::sampling(const Action &u)
{
    /// TODO: propagar todas as particulas de acordo com o modelo de movimento baseado em odometria
    int i;
    double varRot1, varTrans, varRot2;
    double alpha1, alpha2, alpha3, alpha4;

    alpha1 = 1;
    alpha2 = 1;
    alpha3 = 1;
    alpha4 = 1;

    std::normal_distribution<double> samplerRot1(0.01, alpha1*u.rot1 + alpha2*u.trans);
    std::normal_distribution<double> samplerTrans(0.01, alpha3*u.trans + alpha4*(u.rot1 + u.rot2));
    std::normal_distribution<double> samplerRot2(0.01, alpha2*u.rot2 + alpha2*u.trans);

     for(i = 0; i < particles.size(); i++){
           varRot1 = u.rot1 - samplerRot1(*generator);
           varTrans = u.trans - samplerTrans(*generator);
           varRot2 = u.rot2 - samplerRot2(*generator);

           particles.at(i).p.x += varTrans*cos(particles.at(i).p.theta + varRot1);
           particles.at(i).p.y += varTrans*sin(particles.at(i).p.theta + varRot1);
           particles.at(i).p.theta += (varRot1 + varRot2);
      }
}

void MCL::weighting(const std::vector<float> &z)
{
    int i, j;
    float zPart, zRobot, curProb;
    float var = 80;
    float mult, sum;

    sum = 0;
    for(i = 0; i < particles.size(); i++){
       if(particles.at(i).p.x*scale < 0 || particles.at(i).p.x*scale > mapWidth ||
          particles.at(i).p.y*scale < 0 || particles.at(i).p.y*scale > mapHeight){

          particles.at(i).w = 0;
          continue;
    }

    if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] != FREE){
        particles.at(i).w = 0;
        continue;
    }
    mult = 1.0;
    for(j = 0; j < 180; j+= 10){
        zPart = computeExpectedMeasurement(j, particles.at(i).p);
        zRobot = z[j];
        curProb = (1/sqrt(2*PI*var))*exp(-0.5*(pow(zRobot-zPart,2)/var));

        mult *= curProb;
    }
    particles.at(i).w = mult;

        sum += particles.at(i).w;
    }
    if(sum == 0){
        printf("Sum = 0\n");
        for(i = 0; i < particles.size(); i++){
            particles.at(i).w = 1.0/particles.size();
        }
    }
    else{
        for(i = 0; i < particles.size(); i++){
            particles.at(i).w = particles.at(i).w/sum;
        }
    }
}

void MCL::resampling()
{
    std::vector<MCLparticle> nextGeneration;

    float r, c, u;
    int i, j;
    std::uniform_real_distribution<double> samplerU(0, 1.0/particles.size());
    r = samplerU(*generator);
    c = particles.at(0).w;
    i = 0;
    for(j = 1; j <= particles.size(); j++){
        u = r + (float)(1.0/(float)particles.size())*(j-1);
        while(u > c){
            i++;
            if(i >= particles.size()){
                i--;
                break;
            }
            c += particles.at(i).w;
        }
        nextGeneration.push_back(particles.at(i));
    }

    particles = nextGeneration;


}

/////////////////////////////////////////////////////
//// Método Auxiliar para o Modelo de Observacao ////
/////////////////////////////////////////////////////

float MCL::computeExpectedMeasurement(int index, Pose &pose)
{
    double angle = pose.theta + double(90-index)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange;
    }
    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale;

    double i=pose.x*scale;
    double j=pose.y*scale;
    for(int k=0;k<(int)(dist);k++){

        if(mapCells[(int)i][(int)j] == OCCUPIED){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale-(i+deltaX),2)+pow(pose.y*scale-(j+deltaY),2))/scale;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange;
}

//////////////////////////////////
//// Métodos de Inicializacao ////
//////////////////////////////////

void MCL::readMap(std::string mapName)
{
    std::string name("../phir2framework/DiscreteMaps/");
    name += mapName;
    std::ifstream file;
    file.open(name.c_str(), std::ifstream::in);

    if( !file.good() )
    {
        std::cerr << "The file \"" << name << "\"  does not exit!" << std::endl;
        return;
    }

    // Read goal pose
    file >> goal.x >> goal.y;
    std::cout << "Goal x " << goal.x << " y " << goal.y << std::endl;

    // Read dimensions.
    file >> mapWidth >> mapHeight;
    std::cout << "map.width " << mapWidth << " map.height " << mapHeight << std::endl;

    mapCells = new CellOccType*[mapWidth];
        for(int i=0;i<mapWidth;i++)
            mapCells[i] = new CellOccType[mapHeight];

    // Read grid from file.
    char read;
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file >> read;
            switch(read)
            {
                case '1':
                    mapCells[x][y] = OCCUPIED;
                    break;
                case '0':
                    mapCells[x][y] = FREE;
                    break;
                case '-':
                    mapCells[x][y] = UNEXPLORED;
                    break;
            }
        }
    }

    file.close();
}

void MCL::initParticles()
{
    particles.resize(numParticles);

    std::uniform_real_distribution<double> randomX(0.0,mapWidth/scale);
    std::uniform_real_distribution<double> randomY(0.0,mapHeight/scale);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<numParticles; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles[i].p.x = randomX(*generator);
            particles[i].p.y = randomY(*generator);
            particles[i].p.theta = randomTh(*generator);

            // check if particle is valid (known and not obstacle)
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == FREE)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles[i].p.x << ' '
                  << particles[i].p.y << ' '
                  << RAD2DEG(particles[i].p.theta) << std::endl;
    }
}

//////////////////////////////////////////////////////
//// Método de Atualização da Media e Covariancia ////
//////////////////////////////////////////////////////

void MCL::updateMeanAndCovariance()
{
    // Compute Mean
    float sx=0, cx=0;
    meanParticlePose.x = meanParticlePose.y = 0.0;
    for(unsigned int i=0; i<numParticles; i++){
        meanParticlePose.x += particles[i].p.x;
        meanParticlePose.y += particles[i].p.y;
        sx += sin(particles[i].p.theta);
        cx += cos(particles[i].p.theta);
    }
    meanParticlePose.x /= numParticles;
    meanParticlePose.y /= numParticles;
    meanParticlePose.theta = atan2(sx,cx);

    // Compute Covariance Matrix 2x2 (considering only x and y)
    float covariance[2][2];
    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] = 0;

    float diffx, diffy;
    for(unsigned int i=0; i<numParticles; i++){
        diffx  = meanParticlePose.x-particles[i].p.x;
        diffy  = meanParticlePose.y-particles[i].p.y;

        covariance[0][0] += diffx*diffx;    covariance[0][1] += diffx*diffy;
        covariance[1][0] += diffy*diffx;    covariance[1][1] += diffy*diffy;
    }

    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] /= numParticles;

    float T = covariance[0][0] + covariance[1][1]; // Trace
    float D = covariance[0][0]*covariance[1][1] - covariance[0][1]*covariance[1][0]; // Determinant

    if((pow(T,2.0)/4.0 - D)<0.0)
        return;


    float lambda1 = T/2.0 + sqrt(pow(T,2.0)/4.0 - D);
    float lambda2 = T/2.0 - sqrt(pow(T,2.0)/4.0 - D);
    float eigvec1[2], eigvec2[2];

    if(covariance[1][0]!=0.0){
        eigvec1[0] = lambda1 - covariance[1][1];    eigvec2[0] = lambda2 - covariance[1][1];
        eigvec1[1] = covariance[1][0];              eigvec2[1] = covariance[1][0];
    }else if(covariance[0][1]!=0.0){
        eigvec1[0] = covariance[0][1];              eigvec2[0] = covariance[0][1];
        eigvec1[1] = lambda1 - covariance[0][0];    eigvec2[1] = lambda2 - covariance[0][0];
    }else if(covariance[1][0]==0.0 && covariance[0][1]==0.0){
        eigvec1[0] = 1;    eigvec2[0] = 0;
        eigvec1[1] = 0;    eigvec2[1] = 1;
    }


    covAngle = RAD2DEG(atan2(eigvec1[1], eigvec1[0]));

    covMajorAxis = sqrt(lambda1);
    covMinorAxis = sqrt(lambda2);

}

////////////////////////////
//// Métodos de desenho ////
////////////////////////////

void Ellipse(float rx, float ry, float angle, int num_segments=80)
{
    float theta = 2 * M_PI / float(num_segments);
    float c = cos(theta);
    float s = sin(theta);
    float t;

    float x = 1;
    float y = 0;

    glRotatef(angle,0,0,1);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x*rx, y*ry);


        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
    glRotatef(-angle,0,0,1);
}

void MCL::draw()
{

    for(int x=0;x<mapWidth;x++){
        for(int y=0;y<mapHeight;y++){

            if(mapCells[x][y] == OCCUPIED)
                glColor3f(0.0,0.0,0.0);
            else if (mapCells[x][y] == UNEXPLORED)
                glColor3f(0.5,0.5,0.5);
            else
                glColor3f(1.0,1.0,1.0);

            glBegin( GL_QUADS );
            {
                glVertex2f(x  ,y  );
                glVertex2f(x+1,y  );
                glVertex2f(x+1,y+1);
                glVertex2f(x  ,y+1);
            }
            glEnd();
        }
    }

    double dirScale=5;
    glPointSize(4);
    glLineWidth(2);

    float alpha;
    if(transparency)
        alpha = 100.0/numParticles;
    else
        alpha = 1.0;


    for(int p=0;p<particles.size();p++){

        double x=particles[p].p.x*scale;
        double y=particles[p].p.y*scale;
        double th=particles[p].p.theta;


        glColor4f(1.0,0.0,0.0,alpha);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();


        glColor4f(0.0, 0.0, 1.0, alpha);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+dirScale*cos(th), y+dirScale*sin(th));
        }
        glEnd();
    }
    glLineWidth(1);


    double xGoal = goal.x*scale;
    double yGoal = goal.y*scale;
    glTranslatef(xGoal,yGoal,0.0);
    glScalef(1.0/2.0,1.0/2.0,1.0/2.0);
    glColor3f(1.0,0.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-12, -8);
        glVertex2f(  8, 12);
        glVertex2f( 12,  8);
        glVertex2f( -8,-12);
    }
    glEnd();
    glBegin( GL_POLYGON );
    {
        glVertex2f(-12,  8);
        glVertex2f( -8, 12);
        glVertex2f( 12, -8);
        glVertex2f(  8,-12);
    }
    glEnd();
    glScalef(2,2,2);
    glTranslatef(-xGoal,-yGoal,0.0);


    double xRobot = meanParticlePose.x*scale;
    double yRobot = meanParticlePose.y*scale;
    double angRobot = RAD2DEG(meanParticlePose.theta);

    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/5.0,1.0/5.0,1.0/5.0);
    glColor3f(0.0,1.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
    glScalef(5,5,5);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);


    glColor3f(0.0,0.4,0.0);
    glLineWidth(2);
    double chisquare_val = 2.4477;
    glTranslatef(xRobot,yRobot,0.0);
    Ellipse(chisquare_val*covMajorAxis*scale, chisquare_val*covMinorAxis*scale,covAngle);
    glTranslatef(-xRobot,-yRobot,0.0);
    glLineWidth(1);
}
