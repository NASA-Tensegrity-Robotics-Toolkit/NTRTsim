
#include "core/tgModel.h"

// Forward Declaration
class CordeModel;

class tgCordeModel : public tgModel
{
public:
    tgCordeModel();
    
    ~tgCordeModel();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
    
    virtual void step(double dt);
    
    /**
    * Call tgModelVisitor::render() on self and all descendants.
    * @param[in,out] r a reference to a tgModelVisitor
    */
    virtual void onVisit(const tgModelVisitor& r) const;
    
    /** @todo consider adding a toString method **/

private:
    CordeModel* testString;
};
