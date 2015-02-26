<?xml version="1.0" encoding="utf-8"?>
<!-- Edited by XMLSpy® -->
<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:msxsl="urn:schemas-microsoft-com:xslt" >
  <xsl:template match="/">
    <html>
      <head>
        <style type="text/css">

          .header
          {
          BACKGROUND-COLOR: #BEC0C9;
          BORDER-BOTTOM: #ffffff 1px solid;
          BORDER-LEFT: #ffffff 1px solid;
          BORDER-RIGHT: #ffffff 1px solid;
          BORDER-TOP: #ffffff 1px solid;
          COLOR: #000000;
          FONT-WEIGHT: bold
          }
          .infotable
          {
          TEXT-ALIGN: LEFT;
          BACKGROUND-COLOR:  #BEC0C9;
          BORDER-BOTTOM: #ffffff 0px solid;
          BORDER-COLLAPSE: collapse;
          BORDER-LEFT: #ffffff 0px solid;
          BORDER-RIGHT: #ffffff 0px solid;
          BORDER-TOP: #ffffff 0px solid;
          FONT-SIZE: 80%;
          }

          #color1
          {

          BACKGROUND-COLOR: #ffffff;
          }

          #color2
          {

          BACKGROUND-COLOR:#f8f8ff ;
          }

        </style>
      </head>

      <body>

        <table cellspacing="0" width="100%" border="1" bordercolor="white" class="infotable">
          <col width="100" />
          <col width="100" />

          <tr>
            <td nowrap="1" class ="header">Project Information</td>
            <td nowrap="1" class ="header"></td>
          </tr>

          <tr id="color2">
            <td>
              Application
            </td>
            <td >
              <xsl:value-of select="ProjectData/Application/@Name"/>
            </td>
          </tr>              

          <tr>
            <td nowrap="1" class ="header">Device Information</td>
            <td nowrap="1" class ="header"></td>
          </tr>

          <tr id="color1">
            <td>
              Microcontroller
            </td>
            <td >
              <xsl:value-of select="ProjectData/WirelessProjectData/Microcontroller"/>
            </td>
          </tr>
          <tr id="color2">
            <td >
              Transceiver
            </td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/Transceiver"/>
            </td>
          </tr>

          <tr>
            <td nowrap="1" class ="header">Wireless Communication Setup</td>
            <td nowrap="1" class ="header"></td>
          </tr>

          <tr id="color1">
            <td>Frequency (Channel)</td>
            <td >
              <xsl:value-of select="ProjectData/WirelessProjectData/Frequency"/> MHz   
              (Channel <xsl:value-of select="ProjectData/WirelessProjectData/Channel"/>)
            </td>
          </tr>

          <tr id="color2">
            <td >Data Rate</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/DataRate"/> 
              <xsl:value-of select="ProjectData/WirelessProjectData/DataRateUnit"/>
              (ChannelPage <xsl:value-of select="ProjectData/WirelessProjectData/ChannelPage"/>)
            </td>
          </tr>

          <tr id="color1">
            <td >Antenna Diversity</td>
            <td>
              <xsl:choose>
                <xsl:when test="ProjectData/WirelessProjectData/AntennaDiversity != ''">
                  <xsl:value-of select="ProjectData/WirelessProjectData/AntennaDiversity"/>
                </xsl:when>
                <xsl:otherwise>
                  Not Supported
                </xsl:otherwise>
              </xsl:choose>
            </td>
          </tr>

          <!--<tr id="color1">
            <td >IEEE Device Address</td>
            <td>
              <xsl:value-of select="WirelessProjectData/DeviceAddressIeee"/>
            </td>
          </tr>-->

          <tr id="color2">
            <td >Source Device Short Address</td>
            <td>
               0x<xsl:value-of select="ProjectData/WirelessProjectData/DeviceAddressShort"/>
            </td>
          </tr>

          <tr id="color1">
            <td >Pan ID</td>
            <td>
              0x<xsl:value-of select="ProjectData/WirelessProjectData/PanId"/>
            </td>
          </tr>

          <tr id="color1">
            <td >Send data to another device</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/SendDataToAnotherDevice"/>
            </td>
          </tr>

          <xsl:if test="ProjectData/WirelessProjectData/FrameSendMode != ''">
          <tr id="color1">
            <td >Frame Send Mode</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/FrameSendMode"/>
            </td>
          </tr>

          <!--<tr id="color2">
            <td >IEEE Receiver Device Address</td>
            <td>
              <xsl:value-of select="WirelessProjectData/ReceiverDeviceAddressIeee"/>
            </td>
          </tr>-->

          <tr id="color1">
            <td >Receiver Device Short Address</td>
            <td>
              <xsl:choose>
                <xsl:when test="ProjectData/WirelessProjectData/ReceiverDeviceAddressShort != ''">
                  0x<xsl:value-of select="ProjectData/WirelessProjectData/ReceiverDeviceAddressShort"/>
                </xsl:when>
                <xsl:otherwise>
                  0xFFFF
                </xsl:otherwise>
              </xsl:choose>
            </td>
          </tr>

          <tr id="color2">
            <td >Enable Frame Retry</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/FrameRetry"/>
            </td>
          </tr>

          <tr id="color1">
            <td >Enable CSMA-CA</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/CsmaCa"/>
            </td>
          </tr>

          <tr id="color2">
            <td >Request Frame Ack</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/RequestFrameAcq"/>
            </td>
          </tr>
          </xsl:if>
          
          <tr id="color2">
            <td >Receive data from another device</td>
            <td>
              <xsl:value-of select="ProjectData/WirelessProjectData/ReceiveDataFromAnotherDevice"/>
            </td>
          </tr>

          <xsl:if test="ProjectData/WirelessProjectData/FrameReceiveMode != ''">
            <tr id="color1">
              <td >Frame Receive Mode</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/FrameReceiveMode"/>
              </td>
            </tr>
          </xsl:if>
            
          <xsl:if test="ProjectData/WirelessProjectData/SpiPort != ''">
            <tr>
              <td nowrap="1" class ="header">Microcontroller to Transceiver Interface</td>
              <td nowrap="1" class ="header"></td>
            </tr>

            <tr id="color1">
              <td >SPI Port</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/SpiPort"/>
              </td>
            </tr>

            <tr id="color2">
              <td >SEL Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/ChipSelectPortPin"/>
              </td>
            </tr>

            <tr id="color1">
              <td >MOSI Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/MosiPortPin"/>
              </td>
            </tr>

            <tr id="color2">
              <td >MISO Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/MisoPortPin"/>
              </td>
            </tr>

            <tr id="color1">
              <td >SCK Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/SckPortPin"/>
              </td>
            </tr>

            <tr id="color2">
              <td >RST Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/RstPortPin"/>
              </td>
            </tr>

            <tr id="color1">
              <td >IRQ Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/IrqPortPin"/>
              </td>
            </tr>

            <tr id="color2">
              <td >SLP_TR Port Pin</td>
              <td>
                <xsl:value-of select="ProjectData/WirelessProjectData/SlpTrPortPin"/>
              </td>
            </tr>
          </xsl:if>
          
          
          
        </table>
        
      </body>

    </html>
  </xsl:template>
</xsl:stylesheet>

