#ifndef _GRAPHICS_H_
#define _GRAPHICS_H_

#include "main.h"

//ͼ�β���
typedef enum _GraphOperation
{
	Operation_Null=0,
	Operation_Add,
	Operation_Change,
	Operation_Delete
}GraphOperation;

//ͼ����״
typedef enum _GraphShape
{
	Shape_Line=0,
	Shape_Rect,
	Shape_Circle,
	Shape_Oval,
	Shape_Arc,
	Shape_Int,
	Shape_Float,
	Shape_Text
}GraphShape;

//ͼ����ɫ
typedef enum _GraphColor
{
	Color_Self=0,//������ɫ
	Color_Yellow,
	Color_Green,
	Color_Orange,
	Color_Purple,
	Color_Pink,
	Color_Cyan,
	Color_Black,
	Color_White
}GraphColor;

//���ԣ�����Ϊһ����ά����
typedef struct NumPair
{
	int16_t x,y;
}Point,Vector;

//ֱ��
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point startPoint,endPoint;//��ʼ�ͽ�������
}Line;

//����
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point startPoint,endPoint;//��ʼ�ͽ�������
}Rect;

//��Բ
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point centPoint;//Բ������
	uint16_t radius;//�뾶
}Circle;

//��Բ
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point centPoint;//Բ������
	Vector semiaxis;//xy���᳤
}Oval;

//Բ��
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point centPoint;//Բ������
	Vector semiaxis;//xy���᳤
	int16_t startAngle,endAngle;//��ʼ����ֹ�Ƕ�
}Arc;

//������
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point pos;//λ������
	float value;//ֵ
	uint8_t digit;//��Чλ��
	uint16_t size;//�����С
}FloatShape;

//������
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point pos;//λ������
	int32_t value;//ֵ
	uint16_t size;//�����С
}IntShape;

//�ַ���
typedef struct
{
	char *name;//����
	GraphColor color;//��ɫ
	uint8_t width;//��ϸ
	uint8_t layer;//����ͼ����
	
	Point pos;//λ������
	uint8_t data[30];//ֵ
	uint16_t length;//�ַ�����
	uint16_t size;//�����С
}Text;

void Graph_SetLine(Line *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x1,int16_t y1,int16_t x2,int16_t y2);
void Graph_DrawLine(Line *shape,GraphOperation opera);
void Graph_SetRect(Rect *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x1,int16_t y1,int16_t x2,int16_t y2);
void Graph_DrawRect(Rect *shape,GraphOperation opera);
void Graph_SetCircle(Circle *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t radius);
void Graph_DrawCircle(Circle *shape,GraphOperation opera);
void Graph_SetOval(Oval *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t semiaxisX,int16_t semiaxisY);
void Graph_DrawOval(Oval *shape,GraphOperation opera);
void Graph_SetArc(Arc *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int16_t semiaxisX,int16_t semiaxisY,int16_t startAngle,int16_t endAngle);
void Graph_DrawArc(Arc *shape,GraphOperation opera);
void Graph_SetFloat(FloatShape *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,float value,uint8_t digit,uint16_t size);
void Graph_DrawFloat(FloatShape *shape,GraphOperation opera);
void Graph_SetInt(IntShape *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,int32_t value,uint16_t size);
void Graph_DrawInt(IntShape *shape,GraphOperation opera);
void Graph_SetText(Text *shape,char *name,GraphColor color,uint16_t width,uint8_t layer,int16_t x,int16_t y,char *text,uint16_t len,uint16_t size);
void Graph_DrawText(Text *shape,GraphOperation opera);

#endif
