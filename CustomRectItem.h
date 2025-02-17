#pragma once

#include <QGraphicsRectItem>
#include <QBrush>             // For QBrush
#include <QPen>

class CustomRectItem : public QGraphicsRectItem
{
public:
    QBrush colorForType(int type);
    CustomRectItem(qreal x, qreal y, qreal width, qreal height, const int type, QGraphicsItem* parent = nullptr);
    void updateType(int type);
    int _type = 1;
    QPoint pos;
};
