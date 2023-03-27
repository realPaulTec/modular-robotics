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

    color: '#121212'

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

                                onValueChanged: signalHandeler.redSetDilation(value)

                                ToolTip {
                                    visible: redDialSpinBox.hovered
                                    text: 'Dilation'
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

                                onValueChanged: signalHandeler.greenSetDilation(value)

                                ToolTip {
                                    visible: grenenDialSpinBox.hovered
                                    text: 'Dilation'
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

                                onValueChanged: signalHandeler.blueSetDilation(value)

                                ToolTip {
                                    visible: blueDialSpinBox.hovered
                                    text: 'Dilation'
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

                                onValueChanged: signalHandeler.brightSetDilation(value)

                                ToolTip {
                                    visible: brightDialSpinBox.hovered
                                    text: 'Dilation'
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
                spacing: 40
                width: parent.width
                height: 220

                Rectangle {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    color: "#1c1c1c"
                    radius: 10

                    ColumnLayout {
                        anchors.fill: parent

                        ComboBox {
                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            model: [
                                "camera feed",
                                "color corrected",
                                "masked frame",
                                "blue image",
                                "green image",
                                "red image",
                                "white image",
                                "thresholding blue",
                                "thresholding green",
                                "thresholding red",
                                "thresholding white",
                                "combination",
                                "tracking image",
                                "final"                
                            ]

                            currentIndex: 13

                            onCurrentIndexChanged: signalHandeler.dropDown(currentIndex)
                        }

                        RowLayout {
                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6
                            
                            Text {
                                Layout.alignment: Qt.AlignLeft

                                font.pointSize: 12
                                text: 'Use Mask: '
                                color: '#FFFFFF'
                            }

                            Switch {
                                Layout.alignment: Qt.AlignRight
                                position: 0

                                onCheckedChanged: signalHandeler.buttonMaskSelection(position)
                            }
                        }

                        SpinBox {
                            id: erosionSpinBox
                            
                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            value: 4

                            onValueChanged: signalHandeler.erosionSelection(value)

                            ToolTip {
                                visible: erosionSpinBox.hovered
                                text: 'Erosion Steps'
                            }
                        }
                    }
                }

                Rectangle {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    color: "#1c1c1c"
                    radius: 10

                    ColumnLayout {
                        anchors.fill: parent

                        SpinBox {
                            id: differenceSpinBox

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            from: 0
                            to: 500
                            value: 140
                            stepSize: 10

                            onValueChanged: signalHandeler.setMaxDifference(value)

                            ToolTip {
                                visible: differenceSpinBox.hovered
                                text: 'Max Difference'
                            }
                        }

                        SpinBox {
                            id: frequencySpinBox

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            from: 0
                            to: 15

                            value: 3

                            onValueChanged: {
                                signalHandeler.setFrequency(value)
                            }

                            ToolTip {
                                visible: frequencySpinBox.hovered
                                text: 'Frequency (Hz)'
                            }
                        }

                        SpinBox {
                            id: lifetimeSpinBox

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6
                            
                            from: 0
                            to: 100 * 100
                            value: 100
                            stepSize: 5
                            
                            property int decimals: 2
                            property real realValue: value / 100

                            onValueChanged: {
                                signalHandeler.setLifetime(value)
                            }

                            validator: DoubleValidator {
                                bottom: Math.min(lifetimeSpinBox.from, lifetimeSpinBox.to)
                                top:  Math.max(lifetimeSpinBox.from, lifetimeSpinBox.to)
                            }

                            textFromValue: function(value, locale) {
                                return Number(value / 100).toLocaleString(locale, 'f', lifetimeSpinBox.decimals)
                            }

                            valueFromText: function(text, locale) {
                                return Number.fromLocaleString(locale, text) * 100
                            }

                            ToolTip {
                                visible: lifetimeSpinBox.hovered
                                text: 'Component Lifetime (s)'
                            }
                        }
                    }
                }

                Rectangle {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    color: "#1c1c1c"
                    radius: 10

                    ColumnLayout {
                        anchors.fill: parent

                        Slider {
                            id: weightCoordinatesSlider

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            from: 0
                            to: 1

                            value: 0.6 

                            onValueChanged: {
                                signalHandeler.setWeightCoordinates(value)
                            }

                            ToolTip {
                                parent: weightCoordinatesSlider.handle
                                visible: weightCoordinatesSlider.pressed
                                text: "Weight Coordinates: " + weightCoordinatesSlider.value.toFixed(2)
                            }                      
                        }

                        Slider {
                            id: weightBoundsSlider

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            from: 0
                            to: 1

                            value: 1.0 

                            onValueChanged: {
                                signalHandeler.setWeightBounds(value)
                            }

                            ToolTip {
                                parent: weightBoundsSlider.handle
                                visible: weightBoundsSlider.pressed
                                text: "Weight Bounds: " + weightBoundsSlider.value.toFixed(2)
                            }                      
                        }

                        Slider {
                            id: weightAreaSlider

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            from: 0
                            to: 1   

                            value: 0.8  

                            onValueChanged: {
                                signalHandeler.setWeightArea(value)
                            }

                            ToolTip {
                                parent: weightAreaSlider.handle
                                visible: weightAreaSlider.pressed
                                text: "Weight Area: " + weightAreaSlider.value.toFixed(2)
                            }                      
                        }
                    }
                }

                Rectangle {
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    color: "#1c1c1c"
                    radius: 10

                    ColumnLayout {
                        anchors.fill: parent

                        RowLayout {
                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6
                            
                            Text {
                                Layout.alignment: Qt.AlignLeft

                                font.pointSize: 12
                                text: 'Components: '
                                color: '#FFFFFF'
                            }

                            Text {
                                id: textAmountComponents
                                objectName: "textAmountComponents"
                                Layout.alignment: Qt.AlignRight

                                font.pointSize: 12
                                text: 'xxx'
                                color: '#FFFFFF'
                            }
                        }

                        SpinBox {
                            id: frequencyBufferSpinBox

                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6

                            from: 0
                            to: 6

                            value: 2

                            onValueChanged: {
                                signalHandeler.setFrequencyBuffer(value)
                            }

                            ToolTip {
                                visible: frequencyBufferSpinBox.hovered
                                text: 'Frequency Buffer'
                            }
                        }

                        RowLayout {
                            Layout.alignment: Qt.AlignHCenter
                            Layout.preferredWidth: parent.width * 0.6
                            
                            Text {
                                Layout.alignment: Qt.AlignLeft

                                font.pointSize: 12
                                text: 'Frequency: '
                                color: '#FFFFFF'
                            }

                            Text {
                                id: frequencyComponents
                                objectName: "frequencyComponents"
                                Layout.alignment: Qt.AlignRight

                                font.pointSize: 12
                                text: 'xxx'
                                color: '#FFFFFF'
                            }
                        }
                    }
                }
            }
        }
    }
}
