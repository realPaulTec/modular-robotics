import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Controls.Material 2.0
import QtQuick.Layouts 1.14

ApplicationWindow {
    id: applicationWindow

    visible: true
    width: 1430
    height: 800
    title: qsTr("Tracking with tv5")

    minimumWidth: 1440
    minimumHeight: 800

    Material.theme: Material.Dark
    Material.accent: Material.Blue

    color: '#171717'

    Control {
        anchors.fill: parent
        anchors.margins: 40

        ColumnLayout {
            anchors.fill: parent

            RowLayout {
                anchors.top: parent.top

                Image {
                        id: trackingImage
                        fillMode: Image.PreserveAspectFit

                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        
                        source: 'image://imageprovider/trackingImage.png'
                        objectName: 'trackingImage'

                        Timer {
                            id: imageTimer
                            interval: 30
                            repeat: true
                            running: true
                            onTriggered: {
                                trackingImage.source = 'image://imageprovider/secondaryTrackingImage.png'
                                trackingImage.source = 'image://imageprovider/trackingImage.png'
                        }
                    }
                }

                Item {
                    width: 30
                }

                RowLayout {
                    height: trackingImage.height
                    spacing: 20

                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    ButtonGroup { id: trackingColorSelection }

                    Rectangle {
                        id: redRect

                        border.color: '#F3191C'
                        border.width: 1

                        Layout.fillHeight: true

                        color: Qt.rgba(1, 0, 0.2, 0.2)
                        width: 140
                        radius: 10

                        ColumnLayout {
                            anchors.fill: parent

                            RadioButton {
                                id: redRadioButton

                                text: 'RED'
                                height: 80

                                ButtonGroup.group: trackingColorSelection
                                Layout.alignment: Qt.AlignHCenter

                                onClicked: {
                                    signalHandeler.radioSignal(2)
                                }

                                ToolTip {
                                    visible: redRadioButton.hovered
                                    text: 'Track Red'
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#F3191C'

                                Layout.fillWidth: true                        
                            }
                            
                            GridLayout {
                                height: 380
                                
                                RowLayout {
                                    
                                    RangeSlider {
                                        id: redSlider
                                        orientation: Qt.Vertical

                                        from: 0
                                        to: 255

                                        first.value: 240
                                        second.value: 255

                                        Layout.fillHeight: true
                                        Layout.fillWidth: true

                                        first.onValueChanged: {
                                            signalHandeler.redSliderMinSignal(first.value)
                                        }

                                        second.onValueChanged: {
                                            signalHandeler.redSliderMaxSignal(second.value)
                                        }

                                        ToolTip {
                                            parent: redSlider.first.handle
                                            visible: redSlider.first.pressed
                                            text: redSlider.first.value.toFixed(0)
                                        }

                                        ToolTip {
                                            parent: redSlider.second.handle
                                            visible: redSlider.second.pressed
                                            text: redSlider.second.value.toFixed(0)
                                        }
                                    }
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#F3191C'

                                Layout.fillWidth: true                        
                            }

                            SpinBox {
                                id: redDialSpinBox

                                Layout.fillWidth: true
                                value: 6

                                onValueChanged: signalHandeler.redSetDialation(value)

                                ToolTip {
                                    visible: redDialSpinBox.hovered
                                    text: 'Dialation'
                                }
                            }
                        }    
                    }
                    
                    Rectangle {
                        id: greenRect

                        border.color: '#43ff64'
                        border.width: 1

                        Layout.fillHeight: true

                        color: Qt.rgba(0, 1, 0.6, 0.2)
                        width: 140
                        radius: 10
                        
                        ColumnLayout {
                            anchors.fill: parent

                            RadioButton {
                                id: greenRadioButton

                                text: 'GREEN'
                                height: 80

                                ButtonGroup.group: trackingColorSelection
                                Layout.alignment: Qt.AlignHCenter

                                onClicked: {
                                    signalHandeler.radioSignal(1)
                                }

                                ToolTip {
                                    visible: greenRadioButton.hovered
                                    text: 'Track Green'
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#43ff64'

                                Layout.fillWidth: true                        
                            }
                            
                            GridLayout {
                                height: 380
                                
                                RowLayout {
                                    
                                    RangeSlider {
                                        id: greenSlider
                                        orientation: Qt.Vertical

                                        from: 0
                                        to: 255

                                        first.value: 240
                                        second.value: 255

                                        Layout.fillHeight: true
                                        Layout.fillWidth: true

                                        first.onValueChanged: {
                                            signalHandeler.greenSliderMinSignal(first.value)
                                        }

                                        second.onValueChanged: {
                                            signalHandeler.greenSliderMaxSignal(second.value)
                                        }

                                        ToolTip {
                                            parent: greenSlider.first.handle
                                            visible: greenSlider.first.pressed
                                            text: greenSlider.first.value.toFixed(0)
                                        }

                                        ToolTip {
                                            parent: greenSlider.second.handle
                                            visible: greenSlider.second.pressed
                                            text: greenSlider.second.value.toFixed(0)
                                        }
                                    }
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#43ff64'

                                Layout.fillWidth: true                        
                            }

                            SpinBox {
                                id: grenenDialSpinBox

                                Layout.fillWidth: true
                                value: 6

                                onValueChanged: signalHandeler.greenSetDialation(value)

                                ToolTip {
                                    visible: grenenDialSpinBox.hovered
                                    text: 'Dialation'
                                }
                            }
                        }    
                    }

                    Rectangle {
                        id: blueRect

                        border.color: '#00c8ff'
                        border.width: 1

                        Layout.fillHeight: true

                        color: Qt.rgba(0, 0.4, 1, 0.2)
                        width: 140
                        radius: 10
                        
                        ColumnLayout {
                            anchors.fill: parent

                            RadioButton {
                                id: blueRadioButton

                                text: 'BLUE'
                                height: 80

                                ButtonGroup.group: trackingColorSelection
                                Layout.alignment: Qt.AlignHCenter

                                onClicked: {
                                    signalHandeler.radioSignal(0)
                                }
                                
                                checked: true

                                ToolTip {
                                    visible: blueRadioButton.hovered
                                    text: 'Track Blue'
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#00c8ff'

                                Layout.fillWidth: true                        
                            }
                            
                            GridLayout {
                                height: 380
                                
                                RowLayout {
                                    
                                    RangeSlider {
                                        id: blueSlider
                                        orientation: Qt.Vertical

                                        from: 0
                                        to: 255

                                        first.value: 240
                                        second.value: 255

                                        Layout.fillHeight: true
                                        Layout.fillWidth: true

                                        first.onValueChanged: {
                                            signalHandeler.blueSliderMinSignal(first.value)
                                        }

                                        second.onValueChanged: {
                                            signalHandeler.blueSliderMaxSignal(second.value)
                                        }

                                        ToolTip {
                                            parent: blueSlider.first.handle
                                            visible: blueSlider.first.pressed
                                            text: blueSlider.first.value.toFixed(0)
                                        }

                                        ToolTip {
                                            parent: blueSlider.second.handle
                                            visible: blueSlider.second.pressed
                                            text: blueSlider.second.value.toFixed(0)
                                        }
                                    }
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#00c8ff'

                                Layout.fillWidth: true                        
                            }

                            SpinBox {
                                id: blueDialSpinBox

                                Layout.fillWidth: true
                                value: 6

                                onValueChanged: signalHandeler.blueSetDialation(value)

                                ToolTip {
                                    visible: blueDialSpinBox.hovered
                                    text: 'Dialation'
                                }
                            }
                        }    
                    }
                
                    Item {
                        width: 30
                    }

                    Rectangle {
                        id: brightRect

                        border.color: '#FFFFFF'
                        border.width: 1

                        Layout.fillHeight: true

                        color: Qt.rgba(1, 1, 1, 0.2)
                        width: 140
                        radius: 10
                        
                        ColumnLayout {
                            anchors.fill: parent

                            RadioButton {
                                id: brightRadioButton

                                text: 'BRIGHT'
                                height: 80

                                ButtonGroup.group: trackingColorSelection
                                Layout.alignment: Qt.AlignHCenter

                                onClicked: {
                                    signalHandeler.radioSignal(3)
                                }

                                ToolTip {
                                    visible: brightRadioButton.hovered
                                    text: 'Track Bright'
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#FFFFFF'

                                Layout.fillWidth: true                        
                            }
                            
                            GridLayout {
                                height: 380
                                
                                RowLayout {
                                    
                                    RangeSlider {
                                        id: brightSlider
                                        orientation: Qt.Vertical

                                        from: 0
                                        to: 255

                                        first.value: 240
                                        second.value: 255

                                        Layout.fillHeight: true
                                        Layout.fillWidth: true

                                        first.onValueChanged: {
                                            signalHandeler.brightSliderMinSignal(first.value)
                                        }

                                        second.onValueChanged: {
                                            signalHandeler.brightSliderMaxSignal(second.value)
                                        }

                                        ToolTip {
                                            parent: brightSlider.first.handle
                                            visible: brightSlider.first.pressed
                                            text: brightSlider.first.value.toFixed(0)
                                        }

                                        ToolTip {
                                            parent: brightSlider.second.handle
                                            visible: brightSlider.second.pressed
                                            text: brightSlider.second.value.toFixed(0)
                                        }
                                    }
                                }
                            }

                            Rectangle {
                                height: 1
                                color: '#FFFFFF'

                                Layout.fillWidth: true                        
                            }

                            SpinBox {
                                id: brightDialSpinBox

                                Layout.fillWidth: true
                                value: 6

                                onValueChanged: signalHandeler.brightSetDialation(value)

                                ToolTip {
                                    visible: brightDialSpinBox.hovered
                                    text: 'Dialation'
                                }
                            }
                        }    
                    }
                }
            }

            Item {
                height: 20
            }

            RowLayout {
                Layout.fillHeight: true
                anchors.margins: 20

                height: 220

                Rectangle {
                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    color: '#00000000'

                    radius: 20
                    border.color: '#393E46'
                    border.width: 2

                    Control {
                        anchors.fill: parent
                        anchors.margins: 20

                        Column {
                            width: 400
                            height: parent.height / 2
                            spacing: 10
                            
                            ComboBox {
                                Layout.fillWidth: true
                                width: 200

                                model: [
                                    "camera feed",
                                    "color corrected",
                                    "masked frame",
                                    "blue image",
                                    "green image",
                                    "red image",
                                    "white image",
                                    "threshholding blue",
                                    "threshholding green",
                                    "threshholding red",
                                    "threshholding white",
                                    "combination",
                                    "tracking image",
                                    "final"                
                                ]

                                currentIndex: 13

                                onCurrentIndexChanged: signalHandeler.dropDown(currentIndex)
                            }

                            RowLayout {
                                Text {
                                    Layout.alignment: Qt.AlignVCenter
                                    
                                    font.pointSize: 12
                                    text: 'Use Mask: '
                                    color: '#FFFFFF'
                                }

                                Switch {
                                    Layout.alignment: Qt.AlignHCenter
                                    Layout.fillWidth: true

                                    position: 1

                                    onCheckedChanged: signalHandeler.buttonMaskSelection(position)
                                }
                            }

                            RowLayout {
                                SpinBox {
                                    id: erosionSpinBox
                                    
                                    width: 40
                                    
                                    Layout.fillWidth: true
                                    value: 4

                                    onValueChanged: signalHandeler.erosionSelection(value)

                                    ToolTip {
                                        visible: erosionSpinBox.hovered
                                        text: 'Erosion Steps'
                                    }
                                }
                            }
                        }
                    }

                }
            }
        }
    }
}
