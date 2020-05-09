#ifndef MANAGER_H
#define MANAGER_H

namespace OmniTurtle
{
  class OmniTurtleManager
  {
  public:
    virtual void moveForward()             =0;
    virtual void moveBackward()            =0;
    virtual void moveRight()               =0;
    virtual void moveLeft()                =0;
    virtual void moveDiagRightUp()         =0;
    virtual void moveDiagLeftUp()          =0;
    virtual void moveDiagRightDown()       =0;
    virtual void moveDiagLeftDown()        =0;
    virtual void turnAroundClockwise()     =0;
    virtual void turnAroundAnticlockwise() =0;
    virtual void stop()                    =0;

  };

}

#endif // MANAGER_H
