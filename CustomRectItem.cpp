#include "CustomRectItem.h"



QBrush CustomRectItem::colorForType(int type)
{
    switch (type) {
    case 0: return QBrush("#596270");   // obstacle
    case 1: return QBrush(Qt::white);   // free type
    case 2: return QBrush("#EB7800");   // node
    case 3: return QBrush("#106CEB");   // origin
    case 4: return QBrush("#10EBDF");    // path type
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

    ////pen.setJoinStyle(Qt::RoundJoin);
    ////pen.setCapStyle(Qt::RoundCap);
    ////pen.setWidth(6);
    //if (!(type == 2 || type == 3)) {
    //    setPen(Qt::NoPen); // Set no pen for no outline

    //}
    //else {
    //    setPen(Qt::SolidLine); // Set no pen for no outline
    //}
}

void CustomRectItem::updateType(int type) {
    _type = type;
    setBrush(colorForType(type)); // Set the brush with the passed QBrush
    update();
}