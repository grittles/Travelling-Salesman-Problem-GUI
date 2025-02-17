#include "CustomRectItem.h"

QBrush CustomRectItem::colorForType(int type)
{
    switch (type) {
    case 0: return QBrush(Qt::gray);    // obstacle
    case 1: return QBrush(Qt::white);   // free type
    case 2: return QBrush(Qt::red);     // node
    case 3: return QBrush(Qt::yellow);  // origin
    case 4: return QBrush(Qt::cyan);    // path type
    default: return QBrush(Qt::black);  // null type
    }
}

CustomRectItem::CustomRectItem(qreal x, qreal y, qreal width, qreal height, int type, QGraphicsItem* parent)
    : QGraphicsRectItem(x, y, width, height, parent)
{
    _type = type;
    pos = QPoint(x,y);
    setBrush(colorForType(type)); // Set the brush with the passed QBrush
    setPen(Qt::NoPen); // Set no pen for no outline
}

void CustomRectItem::updateType(int type) {
    _type = type;
    setBrush(colorForType(type)); // Set the brush with the passed QBrush
    setPen(Qt::NoPen); // Set no pen for no outline
    update();
}