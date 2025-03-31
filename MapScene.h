#pragma once

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>
#include "CustomRectItem.h"

// i don't actually think I ended up using this for anything
class MapScene : public QGraphicsScene
{
    Q_OBJECT

public:
    MapScene(QObject* parent = nullptr);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

signals:
    void pixelAdded(const QPointF& point);
};
