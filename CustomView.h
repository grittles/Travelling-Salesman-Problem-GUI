#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QScrollBar>
#include <QColor>
#include "CustomRectItem.h"
#include "Grid.h"

// So tired of the laggy placement. Let's keep track of our graphics items ourselves so we don't have to keep scanning the grid.
class hashMap {
    QHash<QPoint, CustomRectItem*> grid;

public:
    void addItem(const QPoint key, CustomRectItem* value) {
        grid[key] = value;
    }

    CustomRectItem* itemAt(const QPoint key) {
        return grid.value(key, nullptr);
    }
};


class CustomView : public QGraphicsView
{
    Q_OBJECT

public:

    Grid* gridPointer = nullptr;
    QGraphicsItem* origin = nullptr;

    static CustomView* getInstance();

    explicit CustomView(QWidget* parent = nullptr);
    ~CustomView();

    void getGrid();

    int roundToNearestMultiple(float n, int multiple);

    QHash<QPoint, CustomRectItem*> m_itemsHash;
    //QVector<CustomRectItem*> m_items;
    int _blockSize = 10;

public slots:
    void resetCanvas(int width, int height, int blockSize);
    void resizeCanvas(int gridToCanvas = false);

    void syncCanvasToGrid(); // is this really necessary?
    void syncGridToCanvas();

signals:

    void a_syncCanvasToGrid();
    void a_syncGridToCanvas();

protected:
    void wheelEvent(QWheelEvent* event) override;  // Handle zooming
    void mousePressEvent(QMouseEvent* event) override;  // Handle panning start
    void mouseMoveEvent(QMouseEvent* event) override;   // Handle panning
    void mouseReleaseEvent(QMouseEvent* event) override; // Stop 

    void keyPressEvent(QKeyEvent* event) override;

    void startPanning(QMouseEvent* event);
    void stopPanning();

    void performPanning(QMouseEvent* event);
    void clickPlace(QMouseEvent* event, int type);
    void handleLeftClick(QMouseEvent* event);
    void handleRightClick(QMouseEvent* event);
    void removeItemsatPoint(QPoint pos);
    void removeItem(QGraphicsItem* item);
    void PlaceBlock(int x, int y, int type);

    void addItemsInRect(const QRectF& rect);
    void removeItemsOutsideRect(const QRectF& rect);

    void pauseInput();
    void resumeInput();

    static bool isInBounds(int x, int y, int maxx, int maxy) {
        return (x >= 0 && x <= maxx && y >= 0 && y <= maxy);
    }

private:
    bool m_isPanning;        // Whether the view is currently being panned
    QPoint m_lastMousePos;   // Store the last mouse position for panning

    // Static instance pointer
    static CustomView* instance;
};