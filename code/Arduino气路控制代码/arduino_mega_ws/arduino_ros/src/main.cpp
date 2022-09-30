#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>
#define CLAW 5
#define DETECT A0

void loosen_claws(void);
void shrink_claws(void);
void detecting(char msg[]);

bool loosen=0;
bool shrink=0;
bool detect=0;

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher pub("chatter_sub", &str_msg);

void messageCb( const std_msgs::String& msg)
{
  if(!strcmp(msg.data,"loosen claw"))
    loosen=1;
  if(!strcmp(msg.data,"shrink claw"))
    shrink=1;
  if(!strcmp(msg.data,"detect"))
    detect=1;

}

ros::Subscriber<std_msgs::String> sub("chatter_pub", messageCb );

void setup()
{
  pinMode(CLAW,OUTPUT);
  pinMode(DETECT,INPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop()
{
  if(loosen==1)
  {
    loosen_claws();
    str_msg.data = "has loosened";
    pub.publish( &str_msg ); 
    loosen=0;
  }
  if(shrink==1)
  {
    shrink_claws();
    str_msg.data = "has shrinked";
    pub.publish( &str_msg ); 
    shrink=0;
  }
  if(detect==1)
  {
    char detect_msg[20];
    detecting(detect_msg);
    str_msg.data = detect_msg;
    pub.publish( &str_msg ); 
    detect=0;
  }
  nh.spinOnce(); 

}

void loosen_claws(void)
{
  digitalWrite(CLAW,LOW);
  delay(1000);
}

void shrink_claws(void)
{
  digitalWrite(CLAW,HIGH);
  delay(1000);
}

void detecting(char msg[])
{
  char temp_str[10];
  int digit;
  digit=analogRead(DETECT);
  strcpy(msg,"has detected ");
  sprintf(temp_str,"%-6d",digit);
  for(int i=0;i<7;i++)
    msg[13+i]=temp_str[i];
}
