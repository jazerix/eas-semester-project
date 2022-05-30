/*
 * generated by Xtext 2.26.0
 */
package dev.faurskov.eas.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.AbstractGenerator
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import java.util.Scanner
import java.net.URL
import dev.faurskov.eas.dSL.Device
import dev.faurskov.eas.dSL.LocatedAt
import dev.faurskov.eas.dSL.WiFi

/**
 * Generates code from your model files on save.
 * 
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
class DSLGenerator extends AbstractGenerator {

	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
		
		val device = resource.allContents.filter(Device).next
		val locatedAt = resource.allContents.filter(LocatedAt).next
		val wifi = resource.allContents.filter(WiFi).next
		
		var configuration = new Scanner(new URL("https://raw.githubusercontent.com/jazerix/eas-semester-project/main/device.ino").openStream(), "UTF-8").useDelimiter("\\A").next();
		
		configuration = configuration.replace("%DEVICE%", device.name)
		.replace("%LATITUDE%", locatedAt.latitude.toString())
		.replace("%LONGITUDE%", locatedAt.longitude.toString())
		.replace("%SSID%", wifi.ssid)
		.replace("%PASS%", wifi.password)
		
		fsa.generateFile(device.name + ".ino", configuration);
	}
	

}