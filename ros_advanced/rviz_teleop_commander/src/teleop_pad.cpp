#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "teleop_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{
    // 创建一个输入topic命名的窗口
    QVBoxLayout* topic_layout = new QVBoxLayout;
    topic_layout->addWidget( new QLabel( "Teleop Topic:" ));
    output_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( output_topic_editor_ );

    // 创建一个输入线速度的窗口
    topic_layout->addWidget( new QLabel( "Linear Velocity:" ));
    output_topic_editor_1 = new QLineEdit;
    topic_layout->addWidget( output_topic_editor_1 );

    // 创建一个输入角速度的窗口
    topic_layout->addWidget( new QLabel( "Angular Velocity:" ));
    output_topic_editor_2 = new QLineEdit;
    topic_layout->addWidget( output_topic_editor_2 );

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addLayout( topic_layout );
    setLayout( layout );

    // 创建一个定时器，用来定时发布消息
    QTimer* output_timer = new QTimer( this );

    // 设置信号与槽的连接
    // 输入topic命名，回车后，调用updateTopic()
    connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));    
    // 输入线速度值，回车后，调用update_Linear_Velocity()    
    connect( output_topic_editor_1, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); 
    // 输入角速度值，回车后，调用update_Angular_Velocity()
    connect( output_topic_editor_2, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));

    // 设置定时器的回调函数，按周期调用sendVel()
    connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

    // 设置定时器的周期，100ms
    output_timer->start( 100 );
}

// 更新线速度值
void TeleopPanel::update_Linear_Velocity()
{
    // 获取输入框内的数据
    QString temp_string = output_topic_editor_1->text();
    
    // 将字符串转换成浮点数
    float lin = temp_string.toFloat();  
    
    // 保存当前的输入值
    linear_velocity_ = lin;
}

// 更新角速度值
void TeleopPanel::update_Angular_Velocity()
{
    QString temp_string = output_topic_editor_2->text();
    float ang = temp_string.toFloat() ;  
    angular_velocity_ = ang;
}

// 更新topic命名
void TeleopPanel::updateTopic()
{
    setTopic( output_topic_editor_->text() );
}

// 设置topic命名
void TeleopPanel::setTopic( const QString& new_topic )
{
    // 检查topic是否发生改变.
    if( new_topic != output_topic_ )
    {
        output_topic_ = new_topic;

        // 如果命名为空，不发布任何信息
        if( output_topic_ == "" )
        {
            velocity_publisher_.shutdown();
        }
        // 否则，初始化publisher
        else
        {
            velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
        }

        Q_EMIT configChanged();
    }
}

// 发布消息
void TeleopPanel::sendVel()
{
    if( ros::ok() && velocity_publisher_ )
    {
        geometry_msgs::Twist msg;
        msg.linear.x = linear_velocity_;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = angular_velocity_;
        velocity_publisher_.publish( msg );
    }
}

// 重载父类的功能
void TeleopPanel::save( rviz::Config config ) const
{
    rviz::Panel::save( config );
    config.mapSetValue( "Topic", output_topic_ );
}

// 重载父类的功能，加载配置数据
void TeleopPanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
    QString topic;
    if( config.mapGetString( "Topic", &topic ))
    {
        output_topic_editor_->setText( topic );
        updateTopic();
    }
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
