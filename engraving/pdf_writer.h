// pdf_writer.h — minimal PDF output for engraving strokes
//
// Writes valid PDF-1.4 with lines of varying width and gray.
// Each stroke is a 2D segment with thickness and gray level.
//
// Usage:
//   PDFWriter pdf("out.pdf", 595, 842);  // A4 portrait in points
//   pdf.strokeLine(x0,y0, x1,y1, 0.5, 0.2);  // width=0.5pt, gray=0.2
//   pdf.save();

#ifndef ENGRAVING_PDF_WRITER_H
#define ENGRAVING_PDF_WRITER_H

#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>

struct PDFWriter {
    std::ostringstream content;  // page content stream (uncompressed)
    double pageW, pageH;
    std::string path;

    PDFWriter(const std::string& p, double w, double h)
        : pageW(w), pageH(h), path(p) {}

    void strokeLine(double x1, double y1, double x2, double y2,
                    double width, double gray) {
        content << width << " w " << gray << " G "
                << x1 << " " << y1 << " m "
                << x2 << " " << y2 << " l S\n";
    }

    // Draw a cubic Bezier segment: P0 → P3 with control points CP1, CP2
    void bezierSegment(double x0, double y0,
                       double cx1, double cy1, double cx2, double cy2,
                       double x3, double y3,
                       double width, double gray) {
        content << width << " w " << gray << " G "
                << x0 << " " << y0 << " m "
                << cx1 << " " << cy1 << " "
                << cx2 << " " << cy2 << " "
                << x3 << " " << y3 << " c S\n";
    }

    void save() {
        std::ofstream f(path, std::ios::binary);
        if (!f) return;

        std::string body = content.str();

        // Offsets: we track object start positions for xref
        size_t off1, off2, off3, off4;

        // Header
        f << "%PDF-1.4\n";

        // Object 1: Catalog
        off1 = f.tellp();
        f << "1 0 obj << /Type /Catalog /Pages 2 0 R >> endobj\n";

        // Object 2: Pages
        off2 = f.tellp();
        f << "2 0 obj << /Type /Pages /Kids [3 0 R] /Count 1 >> endobj\n";

        // Object 3: Page
        off3 = f.tellp();
        f << "3 0 obj << /Type /Page /Parent 2 0 R "
          << "/MediaBox [0 0 " << pageW << " " << pageH << "] "
          << "/Contents 4 0 R >> endobj\n";

        // Object 4: Content stream
        off4 = f.tellp();
        f << "4 0 obj << /Length " << body.size() << " >>\n"
          << "stream\n" << body << "endstream\nendobj\n";

        // xref table
        size_t xrefOff = f.tellp();
        f << "xref\n0 5\n"
          << "0000000000 65535 f \n"
          << std::setfill('0') << std::setw(10) << off1 << " 00000 n \n"
          << std::setfill('0') << std::setw(10) << off2 << " 00000 n \n"
          << std::setfill('0') << std::setw(10) << off3 << " 00000 n \n"
          << std::setfill('0') << std::setw(10) << off4 << " 00000 n \n";

        // trailer
        f << "trailer << /Size 5 /Root 1 0 R >>\n"
          << "startxref\n" << xrefOff << "\n%%EOF\n";
        f.close();
    }
};

#endif // ENGRAVING_PDF_WRITER_H
