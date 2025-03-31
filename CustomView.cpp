#include "CustomView.h"
#include <QGraphicsItem>
#include <QGraphicsScene>
#include "CustomRectItem.h"
#include "GUI.h"

// Define the static instance (This is required for linking to work)
CustomView* CustomView::instance = nullptr;

CustomRectItem* originBlock;

CustomView::CustomView(QWidget* parent)
    : QGraphicsView(parent), m_isPanning(false)
{
    //this->setScene(new QGraphicsScene(this));
    //this->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    // Set drag mode for the view to enable smooth panning
    setDragMode(QGraphicsView::NoDrag);  // We'll implement custom dragging
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);  // Zoom centered on mouse
    setRenderHint(QPainter::Antialiasing, false);  // disable antialiasing cause it makes stuff ugly

    //resetCanvas(width, height, bSize);

    // below code doesn't even do anything
    //QGraphicsScene* scene = new QGraphicsScene(this);
    //this->setScene(scene);
    //scene->setItemIndexMethod(QGraphicsScene::NoIndex);
}

CustomView::~CustomView()
{
    m_itemsHash.clear();
}

//void CustomView::setGUIParent();


int CustomView::roundToNearestMultiple(float n, int multiple)
{
    return std::floor(static_cast<double>(n) / multiple) * multiple; // lol just floor it for pixels
}

CustomView* CustomView::getInstance() {
    if (!instance) {
        instance = new CustomView();
    }
    return instance;
}

void CustomView::getGrid()
{
    gridPointer = QtWidgetsApplication1::getInstance()->getGrid();
}

void CustomView::PlaceBlock(int x, int y, int type) {

    // todo: for more effeciency just separate the palcement or update of blocks as helper functions and call them here
    QPoint snappedPoint(x, y);
    CustomRectItem* existingItem = m_itemsHash.value(snappedPoint, nullptr);
    
    if (existingItem != nullptr) { // update item
        
        if (existingItem->_type == type) {
            return;
        }
        //QString text = QString("Pixel already exists there, overwriting.");
        //QtWidgetsApplication1::getInstance()->WriteConsole(text);

        //CustomRectItem* customItem = dynamic_cast<CustomRectItem*>(existingItem);

        if (type == 3) { // should really have enum'd this
            if (!(origin == nullptr)) {
                CustomRectItem* customItem = dynamic_cast<CustomRectItem*>(origin);
                customItem->updateType(1); // update the old origin as a free block
            }
            origin = existingItem;
        }

        existingItem->updateType(type);
    }
    else { // place item (redundant except for initiating the scene)
        CustomRectItem* block = new CustomRectItem(x, y, _blockSize, _blockSize, type);
        // commented line below glitched tf out so i explicitly set x and y in my custom class, Qt is so awful
        //block->setPos(snappedPoint); // wtf? If I don't explicitly set it, getPos and scenePos return 0,0
        m_itemsHash[snappedPoint] = block;
        scene()->addItem(block);
    }
}

void CustomView::PlaceRandomBlock(int type) {
    int width = gridPointer->getWidth();
    int height = gridPointer->getHeight();

    QRectF sceneRect = this->sceneRect(); // Get the scene rectangle

    int x = roundToNearestMultiple(random_int(sceneRect.x() + _blockSize, sceneRect.right()), _blockSize) - _blockSize;
    int y = roundToNearestMultiple(random_int(sceneRect.y() + _blockSize, sceneRect.bottom()), _blockSize) - _blockSize;
    
    //for (int x = sceneRect.x(); x < sceneRect.right(); x += _blockSize) {  // Assuming each "pixel" is a 10x10 square
    //    for (int y = sceneRect.y(); y < sceneRect.bottom(); y += _blockSize) {

    PlaceBlock(x,y,type);

}

// Remove item by its pointer
// used by the program while resizing the canvas
void CustomView::removeItem(QGraphicsItem* item) {
    if (item == origin) {
        origin = nullptr;
    }

    CustomRectItem* customItem = dynamic_cast<CustomRectItem*>(item);
    QPoint position = customItem->pos;
    m_itemsHash.remove(position); // we can do this one at a time but we can't loop through and remove stuff from a hashtable lol
    scene()->removeItem(item);
}

// not currently used (can be removed later)
void CustomView::removeItemsatPoint(QPoint pos) {
    QGraphicsItem* item = m_itemsHash.value(pos, nullptr);

    if (item != nullptr) {
        removeItem(item);
    }
}

void CustomView::addItemsInRect(const QRectF& rect) {
    for (int x = rect.x(); x < rect.right(); x += _blockSize) {  // Assuming each "pixel" is a 10x10 square
        for (int y = rect.y(); y < rect.bottom(); y += _blockSize) {
            QGraphicsItem* existingItem = m_itemsHash.value(QPoint(x, y), nullptr);
            if (existingItem == nullptr) {
                PlaceBlock(x, y, 1);
            }
        }
    }
}

void CustomView::removeItemsOutsideRect(const QRectF& rect) {
    QList<QGraphicsItem*> items = scene()->items();
    foreach(QGraphicsItem * item, items) {
        CustomRectItem* customItem = dynamic_cast<CustomRectItem*>(item);
        QPoint position = customItem->pos;
        int x = position.x();
        int y = position.y();

        if ((x >= rect.right()) || (y >= rect.bottom())) {
            removeItem(item);
        }
    }
}

void CustomView::pauseInput() {
    scene()->blockSignals(true); // Block signals to prevent unnecessary updates
    QWidget::setEnabled(false);
}

void CustomView::resumeInput() {
    QWidget::setEnabled(true);
    scene()->blockSignals(false);
    scene()->update(); // Manually update the scene once after all changes
}

void CustomView::resizeCanvas(int gridToCanvas) {
    // this is meant to be called inside of ResizeCanvasArgs in GUI.cpp
    pauseInput();

    QRectF sceneRect = this->sceneRect(); // Get the scene rectangle

    removeItemsOutsideRect(sceneRect);
    addItemsInRect(sceneRect);

    if (gridToCanvas == false) syncCanvasToGrid();
    

    resumeInput();
}

void CustomView::syncCanvasToGrid()
{

    // first verify the canvas
    QRectF sceneRect = this->sceneRect(); // Get the scene rectangle
    
    //removeItemsOutsideRect(sceneRect);
    //addItemsInRect(sceneRect);

    int blocksize = _blockSize; // copy value from global blocksize

    // the canvas is "blocksize" times upscaled to the actual grid
    // make sure to add 1 because i starts at 0.
    int width = (sceneRect.right() + 1)/ blocksize;
    int height = (sceneRect.bottom() + 1)/ blocksize;

    pauseInput();

    // resize the actual grid to the new size. This will downsize or upsize and add or delete.
    gridPointer->resizeGrid(width, height); // resize will do nothing if the same dimensions are used
    gridPointer->fullResetPath();

    //QVector<CustomRectItem*> gay = m_items;

    // for every block (pixel), set the cell to the same type.
    for (CustomRectItem* pixel : m_itemsHash) {
        QPoint position = pixel->pos;
        int type = pixel->_type;
        if (type == 4) type = 1;
        //QString text = QString("Pixel of type [%1] at pos: [%2, %3]").arg(pixel->_type).arg(position.x()).arg(position.y());
        //QtWidgetsApplication1::getInstance()->WriteConsole(text);

        gridPointer->setCell(position.x() / blocksize, position.y() / blocksize, type);
    }

    resumeInput();
    // done
}

void CustomView::syncGridToCanvas()
{
    int blocksize = _blockSize;

    // do reverse of sync canvas to grid and upscale the dimensions
    int width = gridPointer->getWidth();
    int height = gridPointer->getHeight();

    QRectF sceneRect = this->sceneRect();
    int sceneWidth = roundToNearestMultiple(sceneRect.right() + 1, blocksize);
    int sceneHeight = roundToNearestMultiple(sceneRect.bottom() + 1, blocksize);

    // this should not happen
    if (!(sceneWidth == width*blocksize && sceneHeight == height*blocksize))
        QtWidgetsApplication1::getInstance()->ResizeCanvasArgs(width, height, true); // resize to grid size 

    std::vector<std::vector<Cell>>* gridCells = gridPointer->getGrid();

    //pauseInput();

    // placeblock will typically replace the type unless it's missing (shouldn't happen) so placeblock is good here
    for (const auto& row : *gridCells) {
        for (const auto& cell : row) {
            PlaceBlock(cell.x * blocksize, cell.y * blocksize, cell.type);
        }
    }
    //resumeInput();
}

void CustomView::resetCanvas(int width, int height, int blockSize) {
    _blockSize = blockSize;

    pauseInput();

    m_itemsHash.clear();
    scene()->clear(); // Clear existing items if necessary

    origin = nullptr;

    for (int x = 0; x < width; x += blockSize) {
        for (int y = 0; y < height; y += blockSize) {
            PlaceBlock(x, y, 1);
        }
    }

    if (gridPointer != nullptr) {
        gridPointer->fullResetPath();
    }
    
    resumeInput();
}

void CustomView::wheelEvent(QWheelEvent* event) {
    static const double minZoom = 0.1;
    static const double maxZoom = 20.0;
    static double currentScale = 1.0;
    const double scaleFactor = 1.15;

    if (event->angleDelta().y() > 0 && currentScale < maxZoom) {
        scale(scaleFactor, scaleFactor);
        currentScale *= scaleFactor;
    }
    else if (event->angleDelta().y() < 0 && currentScale > minZoom) {
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
        currentScale /= scaleFactor;
    }
}

void CustomView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        startPanning(event);
    }
    else {
        QGraphicsView::mousePressEvent(event);
    }
}

void CustomView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        stopPanning();
    }

    else if (event->button() == Qt::RightButton) {
        handleRightClick(event);
    }
    else if (event->button() == Qt::LeftButton) {
        handleLeftClick(event);
    }
    else {
        QGraphicsView::mouseReleaseEvent(event);
    }
}

void CustomView::mouseMoveEvent(QMouseEvent* event) {
    if (m_isPanning) {
        performPanning(event);
    }
    else {
        QGraphicsView::mouseMoveEvent(event);
    }
}

void CustomView::startPanning(QMouseEvent* event) {
    m_isPanning = true;
    m_lastMousePos = event->pos();
    setCursor(Qt::ClosedHandCursor);
}

void CustomView::stopPanning() {
    m_isPanning = false;
    setCursor(Qt::ArrowCursor);
}

void CustomView::performPanning(QMouseEvent* event) {
    QPoint delta = event->pos() - m_lastMousePos;
    m_lastMousePos = event->pos();
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
}

void CustomView::clickPlace(QMouseEvent* event, int type) {
    QPointF clickedPoint = mapToScene(event->pos());
    int snappedX = roundToNearestMultiple(clickedPoint.x(), _blockSize);
    int snappedY = roundToNearestMultiple(clickedPoint.y(), _blockSize);

    /*QString text = QString("Clicked Point: [%1, %2]").arg(clickedPoint.x()).arg(clickedPoint.y());
    QtWidgetsApplication1::getInstance()->WriteConsole(text);

    text = QString("Rounded Point: [%1, %2]").arg(snappedX).arg(snappedY);
    QtWidgetsApplication1::getInstance()->WriteConsole(text);

    text = QString("Array Rounding: [%1, %2]").arg(snappedX / _blockSize).arg(snappedY / _blockSize);
    QtWidgetsApplication1::getInstance()->WriteConsole(text);*/

    // i redefined this after already defining it to resize canvas, fuck sakes
    QRectF sceneRect = this->sceneRect(); // Get the scene rectangle

    int width = QtWidgetsApplication1::getInstance()->sceneWidth - _blockSize;
    int height = QtWidgetsApplication1::getInstance()->sceneHeight - _blockSize;

    if (isInBounds(snappedX, snappedY, width, height)) {
        //int type = QtWidgetsApplication1::getInstance()->getType(); // sets the type to place to this type.
        PlaceBlock(snappedX, snappedY, type);
    }
}

void CustomView::handleLeftClick(QMouseEvent* event) {
    int type = QtWidgetsApplication1::getInstance()->getType(); // sets the type to place to this type.
    clickPlace(event, type);
}

void CustomView::handleRightClick(QMouseEvent* event) {
    clickPlace(event, 1);
}

void CustomView::keyPressEvent(QKeyEvent* event) {
    //if (event->key() == Qt::Key_F1) {
    //    int width = QtWidgetsApplication1::getInstance()->viewWidth;
    //    int height = QtWidgetsApplication1::getInstance()->viewHeight;
    //    int bSize = QtWidgetsApplication1::getInstance()->bSize;

    //    resetCanvas(width, height, bSize);
    //}
    //else {
    //    QGraphicsView::keyPressEvent(event);  // Ensure other key events are processed
    //}

    QGraphicsView::keyPressEvent(event);  // Ensure other key events are processed
}
