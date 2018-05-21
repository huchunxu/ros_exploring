#ifndef PLUGINLIB_TUTORIALS_POLYGON_BASE_H_
#define PLUGINLIB_TUTORIALS_POLYGON_BASE_H_

namespace polygon_base 
{
    
class RegularPolygon
{
public:
    //pluginlib要求构造函数不能带有参数，所以定义initialize来完成需要初始化的工作
    virtual void initialize(double side_length) = 0;

    //计算面积的接口函数
    virtual double area() = 0;

    virtual ~RegularPolygon(){}

protected:
    RegularPolygon(){}
};

};
#endif