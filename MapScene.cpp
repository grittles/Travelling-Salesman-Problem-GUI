#include "MapScene.h"
#include <QGraphicsScene>
#include <QGraphicsItem>

MapScene::MapScene(QObject* parent)
    : QGraphicsScene(parent)
{
}

void MapScene::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    // Call the base class implementation
    QGraphicsScene::mousePressEvent(event);
}
