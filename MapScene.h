#pragma once

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>
#include "CustomRectItem.h"

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
